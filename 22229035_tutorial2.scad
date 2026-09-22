// M5StickC Plus2 → ODrive v3.6 adapter (COMP3046 Week 2 homework).
// Couples a drop-in M5Stick cradle to the PCB outer short-side mechanical
// mounting hole pair at 38.5 mm centres (drawing dimension; teacher:
// "2 holes on the drive"). Not the tray 4-boss matrix (48 mm). Not the
// mid-board electrical pads (NetJ22_1 / NetC59_2). Not the 39 mm =
// 50 − 2×5.5 estimate.
// Week 2 style: attachable() + named_anchor(); cuboid/rect + linear_extrude;
// holes via attach() + xcopies/ycopies; diff()/tag("remove").
// egg() is for the MAXSwerve motor envelope — not this rectangular cradle.
// Units: mm. Print on the Z=0 floor. Do not export STL until Bot PASS.
//
// Round 11: pocket cutter uses p6 Z rounding (m5stick_zrounding=3, edges="Z").
// Both short ends stay full open mouths (cutter still runs past ±Y).
// Round 10 stands: solid +X wall (no Button B mouth); flange_y < cradle_y;
// pad_d 10; M2 on the USB-C mouth (−Y); M3 pitch 38.5. Open top. Z=0 print face.

include <BOSL2/std.scad>
include <BOSL2/screws.scad>

$fn = $preview ? 8 : 128;

function m5stick_dim() = [25, 48, 13.5];           // L, W, H (tutorial p6)
function m5stick_screw_hole() = 2;
function m5stick_screw_hole_offset() = [16, 9.4];  // X spacing, Y from USB end
function m5stick_zrounding() = 3;

function odrive_pcb_dim() = [135.5, 50.0, 1.6];    // L, W, T
function odrive_tab() = 2.5;
function odrive_hole_corner() = 5.5;               // X inset from PCB short edge
function odrive_mount_drill() = 3.2;
// Outer mechanical pair on a short edge (user/drawing 38.50). Do not derive 39.
function pcb_hole_pitch_short() = 38.5;
function tray_dim() = [150, 60, 4];                // L, W, T (preview context)
function standoff_h() = 6;
function standoff_od() = 6;
function matrix_inset() = 6;
function inner_boss_h() = 4;
function inner_boss_od() = 8;

module m5stickc_odrive_adapter(
        m5_l           = m5stick_dim()[0],
        m5_w           = m5stick_dim()[1],
        m5_h           = m5stick_dim()[2],
        wall           = 3,
        floor_t        = 3,
        fit            = 0.4,
        pocket_h       = 8,
        rounding       = 1,
        overlap        = 0.05,
        hole_overshoot = 0.5,   // cutter extends this past each floor face
        pcb_pitch_y    = pcb_hole_pitch_short(),
        m3_d           = 3.4,   // M3 clearance through PCB Ø3.2
        pad_d          = 10,    // X width. Was 12; outer overhang trimmed ~2 mm
        m2_d           = 2.4,
        m3_y_meat      = 2,     // E2: M3 rim to each flange Y edge. Not stretched to the pocket.
        anchor         = BOTTOM,
        spin           = 0,
        orient         = UP
    ) {
    inner_x   = m5_l + fit;                 // pocket width (X). Do not change.
    inner_y   = m5_w + fit;                 // pocket length (Y). Do not change.
    outer_x   = inner_x + 2 * wall;
    // No short-end walls. Cradle Y is the pocket length, so both
    // mouths are the full pocket section (inner_x × pocket_h).
    cradle_y  = inner_y;
    outer_z   = floor_t + pocket_h;
    flange_x  = pad_d;
    // E2: pitch + hole + rim meat. Do not max() this up to cradle_y.
    // pcb_pitch_y + pad_d is 48.5, which is not shorter than the pocket.
    flange_y  = pcb_pitch_y + m3_d + 2 * m3_y_meat;
    assert(flange_y < cradle_y, "E2: flange Y must be shorter than the pocket");
    assert(m3_y_meat >= 2, "E2: need at least 2 mm from the M3 rim to each flange Y edge");
    pocket_cx = flange_x / 2 - wall + outer_x / 2;
    m2_span   = m5stick_screw_hole_offset()[0];
    // 9.4 mm inboard of the USB-C mouth (−Y), not the IR mouth (+Y).
    m2_from_usb = m5stick_screw_hole_offset()[1];
    through_h = floor_t + 2 * hole_overshoot;

    // Print-frame AABB (Z=0 sitting face, flange centred on X=0).
    xmin      = -flange_x / 2;
    xmax      = flange_x / 2 - wall + outer_x;
    overall_x = xmax - xmin;
    // Cradle is the longer Y extent (E2). Do not grow the flange to this.
    overall_y = cradle_y;
    overall_z = outer_z;
    aabb_cx   = (xmin + xmax) / 2;
    aabb_cz   = overall_z / 2;

    // Named anchors in attachable-native (AABB-centred) coordinates.
    anchors = [
        named_anchor("flange",     [-aabb_cx, 0, -aabb_cz + floor_t / 2], UP, 0),
        named_anchor("flange-bot", [-aabb_cx, 0, -aabb_cz], DOWN, 0),
        named_anchor("flange-top", [-aabb_cx, 0, -aabb_cz + floor_t], UP, 0),
        named_anchor("m3-pair",    [-aabb_cx, 0, -aabb_cz], DOWN, 0),
        named_anchor("m3-fwd",     [-aabb_cx, -pcb_pitch_y / 2, -aabb_cz], DOWN, 0),
        named_anchor("m3-back",    [-aabb_cx,  pcb_pitch_y / 2, -aabb_cz], DOWN, 0),
        named_anchor("cradle",     [pocket_cx - aabb_cx, 0, 0], UP, 0),
        named_anchor("cradle-top", [pocket_cx - aabb_cx, 0, aabb_cz], UP, 0),
        named_anchor("m2-pair",    [
            pocket_cx - aabb_cx,
            -inner_y / 2 + m2_from_usb,
            -aabb_cz
        ], DOWN, 0)
    ];

    attachable(
        anchor  = anchor,
        spin    = spin,
        orient  = orient,
        size    = [overall_x, overall_y, overall_z],
        anchors = anchors
    ) {
        move([-aabb_cx, 0, -aabb_cz]) {
            // Flange: 2D rect extruded (Week 2 linear_extrude pattern).
            // M3 pair stays on the rect centre, pitch 38.5. pad_d 10 leaves
            // (pad_d - m3_d) / 2 = 3.3 mm of meat to each trimmed X edge.
            linear_extrude(height=floor_t, convexity=4)
            diff()
            rect([flange_x, flange_y], rounding=rounding) {
                tag("remove")
                    attach(CENTER)
                        ycopies(pcb_pitch_y, n=2)
                            circle(d=m3_d);
            }

            // Cradle hangs off the PCB/tray short edge (+X). Stick body stays off-PCB.
            // attach(CENTER) keeps world axes. attach(FRONT/BACK) would spin a
            // cutter so its wall-thickness axis became the window height.
            right(pocket_cx)
            diff()
            cuboid([outer_x, cradle_y, outer_z],
                   rounding=rounding, edges="Z", anchor=BOTTOM) {
                // Open top + both short mouths. Cutter is the full pocket
                // width and height and runs past both Y ends (no end wall).
                // E1: p6 Z rounding on this cutter (vertical cavity corners).
                // Long walls (X) and the floor stay. Min wall = wall (3 mm).
                tag("remove")
                    attach(CENTER)
                        up(floor_t / 2)
                            cuboid([inner_x,
                                    cradle_y + 2 * overlap,
                                    pocket_h + 2 * overlap],
                                   rounding=m5stick_zrounding(), edges="Z");

                // RIGHT (+X) long wall stays solid for the full pocket height.
                // No Button B / mid-wall mouth (R10).

                // Two Ø2.4 through the floor on the USB-C mouth (−Y).
                // 16 mm apart in X, 9.4 mm inboard of that open end. Not IR.
                tag("remove")
                    attach(CENTER)
                        down(outer_z / 2 - floor_t / 2)
                        fwd(inner_y / 2 - m2_from_usb)
                        xcopies(m2_span, n=2)
                            cyl(d=m2_d, h=through_h);
            }
        }
        children();
    }
}

m5stickc_odrive_adapter();
