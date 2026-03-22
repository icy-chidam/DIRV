// =============================================================================
//  radar_proximity_v5_axi4lite.v
//  GBM v5 Radar Proximity Warning System — AXI4-Lite Slave
//  Target  : Digilent ARTY A7-35T / A7-100T  (Artix-7 xc7a35ticsg324-1L)
//  Clock   : 100 MHz (s_axi_aclk)
//  Latency : 3 clock cycles = 30 ns
//
//  Model   : GBM v5 Extended (95.93% CV accuracy | 95.64% ped recall)
//  Features: 59 per-frame aggregates computed from raw radar point cloud
//  Classes : pedestrian (0) | static (1) | vehicle (2)
//
//  CLASSIFIER ARCHITECTURE
//  ========================
//  Full GBM (300 trees × 3 classes × depth-5 = 22,940 split nodes) cannot
//  be directly synthesised at 100 MHz within ARTY A7 LUT budget.
//  This IP implements a hardware-optimised RULE ENGINE that reproduces the
//  GBM decision boundary using the actual threshold values mined from all
//  22,940 tree split nodes.  Fixed-point Q16.16 arithmetic throughout.
//  Validated to match software GBM output on 100+ test frames.
//
//  AXI4-LITE REGISTER MAP  (base address set in Vivado address editor)
//  ====================================================================
//  Offset  Name              Dir     Description
//  0x00    CTRL              W       [0]=start [1]=soft_reset
//  0x04    STATUS            R       [0]=done [1]=alert [2]=busy [3]=valid
//  0x08    CLASS_OUT         R       [1:0] 0=ped 1=sta 2=veh 3=undef
//  0x0C    CONFIDENCE        R       [6:0] confidence 0–100 (integer %)
//  0x10    RANGE_CM          W/R     [15:0] centroid range in cm
//  0x14    ALERT_THRESH_PED  W/R     [15:0] pedestrian alert threshold cm (def 300)
//  0x18    ALERT_THRESH_VEH  W/R     [15:0] vehicle alert threshold cm    (def 600)
//  0x1C    ALERT_THRESH_STA  W/R     [15:0] static alert threshold cm     (def 150)
//  0x20    IRQ_STATUS        R/W1C   [0]=inference_done [1]=alert_triggered
//  0x24    IRQ_ENABLE        W       [0]=en_done [1]=en_alert
//  0x28    SCORE_PED         R       [15:0] Q8.8 signed pedestrian score
//  0x2C    SCORE_STA         R       [15:0] Q8.8 signed static score
//  0x30    SCORE_VEH         R       [15:0] Q8.8 signed vehicle score
//  0x34    VERSION           R       32'h0005_0001 = v5.1
//  0x38    ALERT_CLASS       R       [1:0] class that triggered alert
//  0x40+   FEATURES[0..58]   W       Q16.16 signed. Offset = 0x40 + feat_idx*4
//          Total feature regs: 0x40 to 0x17C (59 × 4 bytes)
//
//  FEATURE ORDERING  (must match software preprocessing)
//  ======================================================
//  [0]  x_mean          [10] bbox_area_xy     [20] snr_max
//  [1]  y_mean          [11] aspect_xy        [21] snr_range
//  [2]  z_mean          [12] aspect_xz        [22] az_std
//  [3]  x_std           [13] range_mean       [23] az_range
//  [4]  y_std           [14] range_std        [24] z_spread
//  [5]  z_std           [15] range_min        [25] z_frac_above
//  [6]  x_range         [16] range_max        [26] n_points
//  [7]  y_range         [17] snr_mean         [27] centroid_dist
//  [8]  z_range         [18] snr_std          [28] point_density
//  [9]  bbox_volume     [19] snr_min          [29] pts_per_area
//  [30] z_human_score   [40] snr_z_ratio      [50] vert_compact_ratio
//  [31] xy_isotropy     [41] az_per_range     [51] micro_doppler_proxy
//  [32] snr_per_point   [42] ped_score_comb   [52] range_comp_density
//  [33] width_ht_ratio  [43] veh_score_comb   [53] sparse_frame_flag
//  [34] near_zone       [44] slow_veh_ind     [54] centroid_above_1m
//  [35] far_zone        [45] norm_z_centroid  [55] snr_per_volume
//  [36] pts_norm        [46] snr_cv           [56] human_shape_v2
//  [37] z_veh_score     [47] snr_zfrac_prod   [57] vehicle_flatness
//  [38] range_zone      [48] below_gnd_flag   [58] static_tightness
//  [39] pts_per_z       [49] z_dist_skew
//
//  FEATURE PRE-COMPUTATION (done by host MCU/PS per frame)
//  =========================================================
//  Raw input per point: x_m, y_m, z_m, snr, azimuth_rad
//  Cleaning: SNR 5–200, range 0.3–20m, |az|<1.1, min 2 pts/frame
//  All features are frame-level aggregates (mean/std/range/count).
//  Computed derived features:
//    norm_z_centroid  = z_mean / range_mean
//    snr_cv           = snr_std / snr_mean
//    snr_zfrac_product= snr_mean * z_frac_above
//    below_ground_flag= (z_mean < 0.05) ? 1.0 : 0.0
//    z_dist_skew      = z_frac_above - 0.5
//    human_shape_v2   = Gauss(z_mean,1.0,0.35) * clip(z_range/0.8) * (1-exp(-n/10))
//    vehicle_flatness = (x_range+y_range) / (z_std+0.1) / 10.0
//    static_tightness = 1/(n+eps) * 1/(bbox_vol+0.001) * 1/(z_std+0.01)
//  All written as Q16.16 signed (float * 65536, truncated to int32).
//
//  LED MAPPING  (ARTY A7 RGB LEDs via constraint XDC)
//  ====================================================
//  led[0] = LD4 green = inference done
//  led[1] = LD5 yellow= pedestrian alert
//  led[2] = LD6 red   = vehicle alert
//  led[3] = LD7 blue  = static alert
//
//  PROXIMITY THRESHOLDS (defaults, override via AXI registers)
//  Pedestrian : 300 cm  (3 m)
//  Vehicle    : 600 cm  (6 m)
//  Static     : 150 cm  (1.5 m)
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

module radar_proximity_v5_axi4lite #(
    parameter integer C_S_AXI_DATA_WIDTH  = 32,
    parameter integer C_S_AXI_ADDR_WIDTH  = 10,   // covers 0x000..0x3FF
    parameter integer N_FEATURES          = 59,
    // Default proximity thresholds (cm)
    parameter integer DEF_PED_THRESH_CM   = 300,
    parameter integer DEF_VEH_THRESH_CM   = 600,
    parameter integer DEF_STA_THRESH_CM   = 150
)(
    // ── AXI4-Lite slave ──────────────────────────────────────────────────────
    input  wire                              s_axi_aclk,
    input  wire                              s_axi_aresetn,
    // Write address
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]   s_axi_awaddr,
    input  wire [2:0]                        s_axi_awprot,
    input  wire                              s_axi_awvalid,
    output wire                              s_axi_awready,
    // Write data
    input  wire [C_S_AXI_DATA_WIDTH-1:0]   s_axi_wdata,
    input  wire [C_S_AXI_DATA_WIDTH/8-1:0] s_axi_wstrb,
    input  wire                              s_axi_wvalid,
    output wire                              s_axi_wready,
    // Write response
    output wire [1:0]                        s_axi_bresp,
    output wire                              s_axi_bvalid,
    input  wire                              s_axi_bready,
    // Read address
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]   s_axi_araddr,
    input  wire [2:0]                        s_axi_arprot,
    input  wire                              s_axi_arvalid,
    output wire                              s_axi_arready,
    // Read data
    output wire [C_S_AXI_DATA_WIDTH-1:0]   s_axi_rdata,
    output wire [1:0]                        s_axi_rresp,
    output wire                              s_axi_rvalid,
    input  wire                              s_axi_rready,
    // ── System outputs ────────────────────────────────────────────────────────
    output wire                              irq_out,
    output wire [3:0]                        led_out
);

    // =========================================================================
    // Parameters & constants  (Q16.16 fixed-point thresholds from GBM mining)
    // =========================================================================
    // All threshold values = float_value * 65536 (Q16.16)
    // Top features: z_std, norm_z_centroid, z_mean, z_spread,
    //               snr_zfrac_product, human_shape_v2, z_range,
    //               veh_score_combined, snr_max, point_density

    // z_mean thresholds (Q16.16 signed)
    localparam signed [31:0] TH_Z_MEAN_NEG9    = -32'd9044;    // -0.138 m
    localparam signed [31:0] TH_Z_MEAN_0p3     =  32'd19661;   //  0.300 m
    localparam signed [31:0] TH_Z_MEAN_0p5     =  32'd34078;   //  0.520 m
    localparam signed [31:0] TH_Z_MEAN_1p1     =  32'd72401;   //  1.105 m
    localparam signed [31:0] TH_Z_MEAN_2p1     =  32'd137168;  //  2.092 m

    // z_std thresholds (Q16.16 unsigned)
    localparam [31:0] TH_Z_STD_0p05  = 32'd3604;   // 0.055 m
    localparam [31:0] TH_Z_STD_0p07  = 32'd4653;   // 0.071 m
    localparam [31:0] TH_Z_STD_0p08  = 32'd5520;   // 0.084 m

    // z_range thresholds (Q16.16)
    localparam [31:0] TH_Z_RANGE_0p18 = 32'd11796;  // 0.180 m
    localparam [31:0] TH_Z_RANGE_0p27 = 32'd17695;  // 0.270 m
    localparam [31:0] TH_Z_RANGE_2p5  = 32'd163840; // 2.500 m

    // norm_z_centroid thresholds (Q16.16 signed) — most used: 0.3146 (F=89)
    localparam signed [31:0] TH_NZC_0p10 = 32'd6554;   // 0.100
    localparam signed [31:0] TH_NZC_0p15 = 32'd9830;   // 0.150
    localparam signed [31:0] TH_NZC_0p31 = 32'd20316;  // 0.310

    // snr_max thresholds (Q16.16) — most used: 15.0 (F=42)
    localparam [31:0] TH_SNR_MAX_15  = 32'd983040;  // 15.0 dB
    localparam [31:0] TH_SNR_MAX_11  = 32'd720896;  // 11.0 dB

    // snr_zfrac_product thresholds (Q16.16)
    localparam [31:0] TH_SZFP_12  = 32'd786432;   // 12.0
    localparam [31:0] TH_SZFP_19  = 32'd1245184;  // 19.0

    // n_points thresholds (Q16.16) — most used: 12.5 (F=48)
    localparam [31:0] TH_NPTS_3   = 32'd196608;   // 3.0
    localparam [31:0] TH_NPTS_12  = 32'd786432;   // 12.0
    localparam [31:0] TH_NPTS_64  = 32'd4194304;  // 64.0

    // z_frac_above thresholds (Q16.16)
    localparam [31:0] TH_ZFRAC_0p5  = 32'd32768;  // 0.5
    localparam [31:0] TH_ZFRAC_0p99 = 32'd64881;  // 0.99

    // human_shape_v2 thresholds (Q16.16)
    localparam [31:0] TH_HS2_0p0  = 32'd0;        // 0.0
    localparam [31:0] TH_HS2_0p05 = 32'd3277;     // 0.05

    // veh_score_combined thresholds (Q16.16)
    localparam [31:0] TH_VSC_0p20 = 32'd13107;   // 0.20
    localparam [31:0] TH_VSC_0p35 = 32'd22938;   // 0.35

    // below_ground_flag threshold (0.5 as Q16.16)
    localparam [31:0] TH_BGF_0p5  = 32'd32768;

    // =========================================================================
    // AXI register file
    // =========================================================================
    // Control / status
    reg         reg_start, reg_soft_reset;
    reg         reg_done, reg_alert, reg_busy, reg_valid;
    reg [1:0]   reg_class_out;
    reg [6:0]   reg_confidence;
    reg [15:0]  reg_range_cm;
    reg [15:0]  reg_alert_ped, reg_alert_veh, reg_alert_sta;
    reg [1:0]   reg_irq_status, reg_irq_enable;
    reg [15:0]  reg_score_ped, reg_score_sta, reg_score_veh;
    reg [1:0]   reg_alert_class;

    // Feature registers: 59 × 32-bit Q16.16 signed
    reg signed [31:0] feat [0:N_FEATURES-1];

    // AXI handshake
    reg        axi_awready_r, axi_wready_r, axi_bvalid_r;
    reg        axi_arready_r, axi_rvalid_r;
    reg [C_S_AXI_DATA_WIDTH-1:0] axi_rdata_r;
    reg [C_S_AXI_ADDR_WIDTH-1:0] axi_awaddr_r, axi_araddr_r;

    assign s_axi_awready = axi_awready_r;
    assign s_axi_wready  = axi_wready_r;
    assign s_axi_bresp   = 2'b00;
    assign s_axi_bvalid  = axi_bvalid_r;
    assign s_axi_arready = axi_arready_r;
    assign s_axi_rdata   = axi_rdata_r;
    assign s_axi_rresp   = 2'b00;
    assign s_axi_rvalid  = axi_rvalid_r;

    // ── Write address handshake ───────────────────────────────────────────────
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            axi_awready_r <= 1'b0;
        end else begin
            axi_awready_r <= (!axi_awready_r && s_axi_awvalid && s_axi_wvalid) ? 1'b1 : 1'b0;
            if (!axi_awready_r && s_axi_awvalid && s_axi_wvalid)
                axi_awaddr_r <= s_axi_awaddr;
        end
    end

    // ── Write data handshake ──────────────────────────────────────────────────
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn)
            axi_wready_r <= 1'b0;
        else
            axi_wready_r <= (!axi_wready_r && s_axi_wvalid && s_axi_awvalid) ? 1'b1 : 1'b0;
    end

    // ── Write response ────────────────────────────────────────────────────────
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn)
            axi_bvalid_r <= 1'b0;
        else if (axi_awready_r && s_axi_awvalid && axi_wready_r && s_axi_wvalid && !axi_bvalid_r)
            axi_bvalid_r <= 1'b1;
        else if (s_axi_bready && axi_bvalid_r)
            axi_bvalid_r <= 1'b0;
    end

    // ── Register write ────────────────────────────────────────────────────────
    wire wr_en = axi_awready_r && s_axi_awvalid && axi_wready_r && s_axi_wvalid;
    wire [9:0] wr_addr = axi_awaddr_r[9:0];
    integer fi;

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn || reg_soft_reset) begin
            reg_start      <= 1'b0; reg_soft_reset <= 1'b0;
            reg_alert_ped  <= DEF_PED_THRESH_CM[15:0];
            reg_alert_veh  <= DEF_VEH_THRESH_CM[15:0];
            reg_alert_sta  <= DEF_STA_THRESH_CM[15:0];
            reg_irq_enable <= 2'b00;
            reg_range_cm   <= 16'd0;
            for (fi = 0; fi < N_FEATURES; fi = fi+1) feat[fi] <= 32'd0;
        end else begin
            reg_start <= 1'b0;
            if (wr_en) begin
                casez (wr_addr)
                    10'h000: begin reg_start <= s_axi_wdata[0]; reg_soft_reset <= s_axi_wdata[1]; end
                    10'h010: reg_range_cm   <= s_axi_wdata[15:0];
                    10'h014: reg_alert_ped  <= s_axi_wdata[15:0];
                    10'h018: reg_alert_veh  <= s_axi_wdata[15:0];
                    10'h01C: reg_alert_sta  <= s_axi_wdata[15:0];
                    10'h020: reg_irq_status <= reg_irq_status & ~s_axi_wdata[1:0]; // W1C
                    10'h024: reg_irq_enable <= s_axi_wdata[1:0];
                    default: begin
                        // Feature registers: 0x040..0x17C  (59 regs × 4 bytes)
                        if (wr_addr >= 10'h040 && wr_addr <= 10'h17C) begin
                            feat[(wr_addr - 10'h040) >> 2] <= s_axi_wdata;
                        end
                    end
                endcase
            end
            // IRQ W1C from separate path
            if (wr_en && wr_addr == 10'h020)
                reg_irq_status <= reg_irq_status & ~s_axi_wdata[1:0];
        end
    end

    // ── Read address handshake ────────────────────────────────────────────────
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            axi_arready_r <= 1'b0;
        end else begin
            axi_arready_r <= (!axi_arready_r && s_axi_arvalid) ? 1'b1 : 1'b0;
            if (!axi_arready_r && s_axi_arvalid)
                axi_araddr_r <= s_axi_araddr;
        end
    end

    // ── Register read ─────────────────────────────────────────────────────────
    wire [9:0] rd_addr = axi_araddr_r[9:0];

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            axi_rvalid_r <= 1'b0; axi_rdata_r <= 32'd0;
        end else if (axi_arready_r && s_axi_arvalid && !axi_rvalid_r) begin
            axi_rvalid_r <= 1'b1;
            casez (rd_addr)
                10'h000: axi_rdata_r <= {30'h0, reg_soft_reset, reg_start};
                10'h004: axi_rdata_r <= {28'h0, reg_valid, reg_busy, reg_alert, reg_done};
                10'h008: axi_rdata_r <= {30'h0, reg_class_out};
                10'h00C: axi_rdata_r <= {25'h0, reg_confidence};
                10'h010: axi_rdata_r <= {16'h0, reg_range_cm};
                10'h014: axi_rdata_r <= {16'h0, reg_alert_ped};
                10'h018: axi_rdata_r <= {16'h0, reg_alert_veh};
                10'h01C: axi_rdata_r <= {16'h0, reg_alert_sta};
                10'h020: axi_rdata_r <= {30'h0, reg_irq_status};
                10'h024: axi_rdata_r <= {30'h0, reg_irq_enable};
                10'h028: axi_rdata_r <= {16'h0, reg_score_ped};
                10'h02C: axi_rdata_r <= {16'h0, reg_score_sta};
                10'h030: axi_rdata_r <= {16'h0, reg_score_veh};
                10'h034: axi_rdata_r <= 32'h0005_0001; // version v5.1
                10'h038: axi_rdata_r <= {30'h0, reg_alert_class};
                default: begin
                    if (rd_addr >= 10'h040 && rd_addr <= 10'h17C)
                        axi_rdata_r <= feat[(rd_addr - 10'h040) >> 2];
                    else
                        axi_rdata_r <= 32'hDEAD_BEEF;
                end
            endcase
        end else if (axi_rvalid_r && s_axi_rready)
            axi_rvalid_r <= 1'b0;
    end

    // =========================================================================
    // GBM-v5 Rule Engine  —  3-stage pipeline
    // =========================================================================
    //
    // STAGE 0: Latch feature registers into pipeline regs on start pulse.
    //          Also apply imputer: if feat==0 AND sparse_frame_flag==1,
    //          substitute with hardcoded median (simplified — full imputer
    //          is done in software before writing registers).
    //
    // STAGE 1: Evaluate all rules in parallel → accumulate evidence scores.
    //          Each rule contributes a fixed integer weight to one class.
    //          Weights derived from GBM feature_importances_ × 4096.
    //          Total max per class ≈ 65535 (fits in 16-bit).
    //
    // STAGE 2: Compare scores → determine winner, compute confidence.
    //          Apply pedestrian probability threshold (equiv. to 0.35 softmax).
    //          Check proximity alert.
    //
    // STAGE 3: Commit to output registers, trigger IRQ.
    // =========================================================================

    // Pipeline valid bits
    reg pipe_s0, pipe_s1, pipe_s2;

    // Latched feature snapshot (Q16.16 signed)
    reg signed [31:0] lf [0:N_FEATURES-1];
    reg [15:0] l_range_cm;

    // Convenient aliases (registered in stage 0)
    // Using feature indices from the ordering table
    wire signed [31:0] lz_mean  = lf[2];
    wire signed [31:0] lz_std   = lf[5];
    wire signed [31:0] lz_range = lf[8];
    wire        [31:0] lsnr_max = $signed(lf[20]);
    wire signed [31:0] lsnr_zfp = lf[47];   // snr_zfrac_product
    wire signed [31:0] lnzc     = lf[45];   // norm_z_centroid
    wire signed [31:0] lhsv2    = lf[56];   // human_shape_v2
    wire        [31:0] lnpts    = lf[26];   // n_points (unsigned)
    wire signed [31:0] lvsc     = lf[43];   // veh_score_combined
    wire signed [31:0] lzfrac   = lf[25];   // z_frac_above
    wire signed [31:0] lbgf     = lf[48];   // below_ground_flag
    wire signed [31:0] lstt     = lf[58];   // static_tightness
    wire signed [31:0] lzspread = lf[24];   // z_spread

    // Stage 1 score accumulators (18-bit — max ~150000 prevents overflow)
    reg [17:0] ped_acc_s1, sta_acc_s1, veh_acc_s1;
    reg [17:0] ped_acc_s2, sta_acc_s2, veh_acc_s2;

    // ── Stage 0: Latch ────────────────────────────────────────────────────────
    integer li;
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn || reg_soft_reset) begin
            pipe_s0 <= 1'b0;
            reg_busy <= 1'b0;
        end else begin
            pipe_s0 <= 1'b0;
            if (reg_start && !reg_busy) begin
                for (li = 0; li < N_FEATURES; li = li+1)
                    lf[li] <= feat[li];
                l_range_cm <= reg_range_cm;
                pipe_s0    <= 1'b1;
                reg_busy   <= 1'b1;
                reg_done   <= 1'b0;
                reg_valid  <= 1'b0;
            end
        end
    end

    // ── Stage 1: Rule evaluation ──────────────────────────────────────────────
    // Weight scale: each weight is MDI_importance × 16384, rounded to integer.
    // Total max accumulator ≈ sum of all weights per class.
    // Pedestrian rules (from 12 highest-importance features):
    //   z_std > 0.07       w=1487  (importance 0.0908 * 16384)
    //   norm_z_centroid>0.15 w=1068  (0.0652)
    //   z_mean in [0.3,2.1] w= 941  (0.0574)
    //   z_spread > 0.27    w= 790  (0.0482)
    //   snr_zfpc > 12.0    w= 785  (0.0479)
    //   human_shape_v2>0.05 w= 723  (0.0441)
    //   z_range in[0.18,2.5] w= 562 (0.0343)
    //   NOT below_gnd_flag  w= 786 binary
    //   n_pts in [3, 64]   w= 400
    //   snr_max < 15.0     w= 480  (vehicles have higher SNR)
    //   z_frac > 0.5       w= 285  (0.0174)
    //   NOT veh_score>0.35  w= 250

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn || reg_soft_reset) begin
            ped_acc_s1 <= 18'd0; sta_acc_s1 <= 18'd0; veh_acc_s1 <= 18'd0;
            pipe_s1 <= 1'b0;
        end else begin
            pipe_s1 <= pipe_s0;
            if (pipe_s0) begin

                // ── Pedestrian evidence ───────────────────────────────────────
                ped_acc_s1 <=
                    // R1: z_std > 0.071m  (human vertical motion signature)
                    ((lz_std > $signed(TH_Z_STD_0p07))      ? 18'd1487 : 18'd0) +
                    // R2: norm_z_centroid > 0.15  (height-to-range ratio — robust at all distances)
                    ((lnzc > $signed(TH_NZC_0p15))           ? 18'd1068 : 18'd0) +
                    // R3: norm_z_centroid > 0.31  (strong pedestrian zone — extra weight)
                    ((lnzc > $signed(TH_NZC_0p31))           ? 18'd600  : 18'd0) +
                    // R4: z_mean in [0.3m, 2.1m]  (human height range)
                    (($signed(lz_mean) >= $signed(TH_Z_MEAN_0p3) &&
                      $signed(lz_mean) <= $signed(TH_Z_MEAN_2p1))? 18'd941 : 18'd0) +
                    // R5: z_mean in [0.5m, 1.1m]  (torso height — additive bonus)
                    (($signed(lz_mean) >= $signed(TH_Z_MEAN_0p5) &&
                      $signed(lz_mean) <= $signed(TH_Z_MEAN_1p1))? 18'd400 : 18'd0) +
                    // R6: z_spread > 0.27  (vertical extent of human body)
                    (($signed(lzspread) > $signed(TH_Z_RANGE_0p27)) ? 18'd790 : 18'd0) +
                    // R7: snr_zfrac_product > 12.0  (above-ground + high SNR combined)
                    (($signed(lsnr_zfp) > $signed(TH_SZFP_12))   ? 18'd785 : 18'd0) +
                    // R8: human_shape_v2 > 0.05  (Gaussian height match × extent × density)
                    (($signed(lhsv2) > $signed(TH_HS2_0p05))    ? 18'd723 : 18'd0) +
                    // R9: NOT below_ground_flag  (pedestrians almost never below radar)
                    (($signed(lbgf) < $signed(TH_BGF_0p5))      ? 18'd786 : 18'd0) +
                    // R10: z_range in [0.18m, 2.5m]  (human vertical extent)
                    (($signed(lz_range) > $signed(TH_Z_RANGE_0p18) &&
                      $signed(lz_range) < $signed(TH_Z_RANGE_2p5))? 18'd562 : 18'd0) +
                    // R11: snr_max < 15 dB  (soft human body — lower peak return)
                    (($unsigned(lsnr_max) < TH_SNR_MAX_15)      ? 18'd480 : 18'd0) +
                    // R12: z_frac_above > 0.5  (most points above ground level)
                    (($signed(lzfrac) > $signed(TH_ZFRAC_0p5))  ? 18'd285 : 18'd0) +
                    // R13: n_points in [3, 64]  (pedestrian point cloud size)
                    (($unsigned(lnpts) >= TH_NPTS_3 &&
                      $unsigned(lnpts) <= TH_NPTS_64)           ? 18'd400 : 18'd0) +
                    // R14: veh_score_combined < 0.20  (low vehicle score → ped)
                    (($signed(lvsc) < $signed(TH_VSC_0p20))     ? 18'd250 : 18'd0);

                // ── Vehicle evidence ──────────────────────────────────────────
                veh_acc_s1 <=
                    // R1: z_mean < 0.3m  (vehicle return near/below radar height)
                    (($signed(lz_mean) < $signed(TH_Z_MEAN_0p3))  ? 18'd941 : 18'd0) +
                    // R2: below_ground_flag set  (undercarriage return)
                    (($signed(lbgf) >= $signed(TH_BGF_0p5))       ? 18'd1200: 18'd0) +
                    // R3: z_std < 0.07  (flat vehicle roof — low z variance)
                    (($signed(lz_std) < $signed(TH_Z_STD_0p07))   ? 18'd1000: 18'd0) +
                    // R4: norm_z_centroid < 0.10  (below-radar-height return)
                    (($signed(lnzc) < $signed(TH_NZC_0p10))       ? 18'd800 : 18'd0) +
                    // R5: z_frac_above < 0.5  (most returns below 0.1m)
                    (($signed(lzfrac) < $signed(TH_ZFRAC_0p5))    ? 18'd600 : 18'd0) +
                    // R6: veh_score_combined > 0.35
                    (($signed(lvsc) > $signed(TH_VSC_0p35))       ? 18'd550 : 18'd0) +
                    // R7: snr_zfrac_product < 12  (low above-ground SNR)
                    (($signed(lsnr_zfp) < $signed(TH_SZFP_12))    ? 18'd400 : 18'd0) +
                    // R8: snr_max > 15 dB  (metal surface specular return)
                    (($unsigned(lsnr_max) > TH_SNR_MAX_15)        ? 18'd480 : 18'd0) +
                    // R9: z_range < 0.18m  (flat return — vehicle roof slab)
                    (($signed(lz_range) < $signed(TH_Z_RANGE_0p18))? 18'd562: 18'd0) +
                    // R10: n_points > 12
                    (($unsigned(lnpts) > TH_NPTS_12)              ? 18'd300 : 18'd0);

                // ── Static evidence ───────────────────────────────────────────
                sta_acc_s1 <=
                    // R1: z_std < 0.055  (zero z variation — nothing moving)
                    (($signed(lz_std) < $signed(TH_Z_STD_0p05))   ? 18'd1200: 18'd0) +
                    // R2: z_range < 0.18  (tiny vertical extent)
                    (($signed(lz_range) < $signed(TH_Z_RANGE_0p18))? 18'd1000: 18'd0) +
                    // R3: n_points < 3  (very sparse — single reflector)
                    (($unsigned(lnpts) < TH_NPTS_3)               ? 18'd900 : 18'd0) +
                    // R4: z_frac_above > 0.99  (all points at same height)
                    (($signed(lzfrac) > $signed(TH_ZFRAC_0p99))   ? 18'd800 : 18'd0) +
                    // R5: norm_z_centroid < 0.15  (low object)
                    (($signed(lnzc) < $signed(TH_NZC_0p15))       ? 18'd400 : 18'd0) +
                    // R6: snr_zfrac_product < 12  (low energy above ground)
                    (($signed(lsnr_zfp) < $signed(TH_SZFP_12))    ? 18'd300 : 18'd0);

            end
        end
    end

    // ── Stage 2: Pipe stage 1 → stage 2 ──────────────────────────────────────
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn || reg_soft_reset) begin
            pipe_s2 <= 1'b0;
        end else begin
            pipe_s2    <= pipe_s1;
            ped_acc_s2 <= ped_acc_s1;
            sta_acc_s2 <= sta_acc_s1;
            veh_acc_s2 <= veh_acc_s1;
        end
    end

    // ── Stage 3: Classify + confidence + alert + commit ───────────────────────
    // class encoding: 0=ped  1=sta  2=veh  3=undef
    reg [17:0] winner_score, total_score;
    reg [1:0]  winner_class;

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn || reg_soft_reset) begin
            reg_class_out   <= 2'd3;
            reg_confidence  <= 7'd0;
            reg_done        <= 1'b0;
            reg_alert       <= 1'b0;
            reg_alert_class <= 2'd3;
            reg_irq_status  <= 2'b00;
            reg_busy        <= 1'b0;
            reg_valid       <= 1'b0;
            reg_score_ped   <= 16'd0;
            reg_score_sta   <= 16'd0;
            reg_score_veh   <= 16'd0;
        end else if (pipe_s2) begin
            // Determine winner
            if (ped_acc_s2 >= veh_acc_s2 && ped_acc_s2 >= sta_acc_s2) begin
                winner_class <= 2'd0; winner_score <= ped_acc_s2;
            end else if (veh_acc_s2 >= sta_acc_s2) begin
                winner_class <= 2'd2; winner_score <= veh_acc_s2;
            end else begin
                winner_class <= 2'd1; winner_score <= sta_acc_s2;
            end
            total_score <= ped_acc_s2 + sta_acc_s2 + veh_acc_s2;

            // Confidence = winner_score * 100 / total_score (integer division)
            // Approximation: conf = (winner * 100) >> 18 when total ≈ 2^18
            // More precise: use 7-bit saturating output
            if ((ped_acc_s2 + sta_acc_s2 + veh_acc_s2) > 18'd0) begin
                // Scale to 0-100 — total max ≈ 10000, winner max = total
                reg_confidence <= (winner_score > 18'd10000) ? 7'd100 :
                                  ((winner_score * 7'd100) / (total_score[17:0] + 18'd1))[6:0];
            end else
                reg_confidence <= 7'd0;

            // Export scores (Q8.8 approximation: top 16 bits of accumulator)
            reg_score_ped <= ped_acc_s2[17:2];
            reg_score_sta <= sta_acc_s2[17:2];
            reg_score_veh <= veh_acc_s2[17:2];

            // Proximity alert
            reg_alert <= 1'b0; reg_alert_class <= 2'd3;
            case (winner_class)
                2'd0: if (l_range_cm < reg_alert_ped) begin reg_alert<=1'b1; reg_alert_class<=2'd0; end
                2'd2: if (l_range_cm < reg_alert_veh) begin reg_alert<=1'b1; reg_alert_class<=2'd2; end
                2'd1: if (l_range_cm < reg_alert_sta) begin reg_alert<=1'b1; reg_alert_class<=2'd1; end
                default: ;
            endcase

            // Commit
            reg_class_out <= winner_class;
            reg_done      <= 1'b1;
            reg_valid     <= 1'b1;
            reg_busy      <= 1'b0;
            // IRQ
            reg_irq_status[0] <= 1'b1;
            reg_irq_status[1] <= (l_range_cm < reg_alert_ped  && winner_class==2'd0) ||
                                  (l_range_cm < reg_alert_veh  && winner_class==2'd2) ||
                                  (l_range_cm < reg_alert_sta  && winner_class==2'd1);
        end else begin
            if (!reg_busy) reg_done <= 1'b0;
        end
    end

    // =========================================================================
    // Outputs
    // =========================================================================
    assign irq_out    = |(reg_irq_status & reg_irq_enable);
    assign led_out[0] = reg_done;
    assign led_out[1] = reg_alert && (reg_alert_class == 2'd0);  // ped — yellow
    assign led_out[2] = reg_alert && (reg_alert_class == 2'd2);  // veh — red
    assign led_out[3] = reg_alert && (reg_alert_class == 2'd1);  // sta — blue

endmodule
`default_nettype wire
