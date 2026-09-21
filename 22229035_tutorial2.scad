// M5StickC Plus2 → ODrive v3.6 adapter (COMP3046 Week 2 homework).
// Couples a drop-in M5Stick cradle to the PCB outer short-side mechanical
// mounting hole pair at 38.5 mm centres (drawing dimension; teacher:
// "2 holes on the drive"). Not the tray 4-boss matrix (48 mm). Not the
// mid-board electrical pads (NetJ22_1 / NetC59_2). Not the 39 mm =
// 50 − 2×5.5 estimate.
// Week 2 style: attachable() + named_anchor(); cuboid/rect + linear_extrude;
// holes via attach() + xcopies/ycopies; diff()/tag("remove").
// egg() is for the MAXSwerve motor envelope — not this rectangular cradle.
// Units: mm. Print on the Z=0 floor.

include <BOSL2/std.scad>
include <BOSL2/screws.scad>

$fn = $preview ? 8 : 128;

function m5stick_dim() = [25, 48, 13.5];           // L, W, H (tutorial p6)
function m5stick_screw_hole() = 2;
function m5stick_screw_hole_offset() = [16, 9.4];  // X spacing, Y from inner FWD
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
        pad_d          = 12,
        m2_d           = 2.4,
        usb_cut_w      = 18,
        usb_cut_h      = 8,
        ir_cut_w       = 10,
        ir_cut_h       = 6,
        btn_cut_w      = 12,
        btn_cut_h      = 8,
        anchor         = BOTTOM,
        spin           = 0,
        orient         = UP
    ) {
    inner_x     = m5_l + fit;
    inner_y     = m5_w + fit;
    outer_x     = inner_x + 2 * wall;
    outer_y     = inner_y + 2 * wall;
    outer_z     = floor_t + pocket_h;
    flange_x    = pad_d;
    flange_y    = max(pcb_pitch_y + pad_d, outer_y);
    pocket_cx   = flange_x / 2 - wall + outer_x / 2;
    m2_span     = m5stick_screw_hole_offset()[0];
    m2_from_fwd = m5stick_screw_hole_offset()[1];
    usb_h       = min(usb_cut_h, pocket_h);
    ir_h        = min(ir_cut_h, pocket_h);
    btn_h       = min(btn_cut_h, pocket_h);
    through_h   = floor_t + 2 * hole_overshoot;

    // Print-frame AABB (Z=0 sitting face, flange centred on X=0).
    xmin      = -flange_x / 2;
    xmax      = flange_x / 2 - wall + outer_x;
    overall_x = xmax - xmin;
    overall_y = max(flange_y, outer_y);
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
            -outer_y / 2 + wall + m2_from_fwd,
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
            linear_extrude(height=floor_t, convexity=4)
            diff()
            rect([flange_x, flange_y], rounding=rounding) {
                tag("remove")
                    attach(CENTER)
                        ycopies(pcb_pitch_y, n=2)
                            circle(d=m3_d);
            }

            // Cradle hangs off the PCB/tray short edge (+X). Stick body stays off-PCB.
            right(pocket_cx)
            diff()
            cuboid([outer_x, outer_y, outer_z],
                   rounding=rounding, edges="Z", anchor=BOTTOM) {
                tag("remove")
                    attach(TOP, TOP, inside=true, overlap=overlap)
                        cuboid([inner_x, inner_y, pocket_h + overlap],
                               rounding=m5stick_zrounding(), edges="Z",
                               anchor=BOTTOM);

                // USB-C + Grove (FWD short wall).
                tag("remove")
                    attach(FRONT, overlap=overlap)
                        up(floor_t + usb_h / 2 - outer_z / 2)
                            cuboid([usb_cut_w, wall + 2 * overlap, usb_h]);

                // IR / alternate port (BACK short wall).
                tag("remove")
                    attach(BACK, overlap=overlap)
                        up(floor_t + ir_h / 2 - outer_z / 2)
                            cuboid([ir_cut_w, wall + 2 * overlap, ir_h]);

                // Button B, off-tray long wall (RIGHT).
                tag("remove")
                    attach(RIGHT, overlap=overlap)
                        up(floor_t + btn_h / 2 - outer_z / 2)
                            cuboid([wall + 2 * overlap, btn_cut_w, btn_h]);

                // Tutorial M2 pattern through the floor (optional stick clamp).
                tag("remove")
                    attach(CENTER)
                        down(outer_z / 2 - floor_t / 2)
                        fwd(outer_y / 2 - wall - m2_from_fwd)
                        xcopies(m2_span, n=2)
                            cyl(d=m2_d, h=through_h);
            }
        }
        children();
    }
}

m5stickc_odrive_adapter();
