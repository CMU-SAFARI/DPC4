// Auto-generated Bandit Configuration Source
#include "umama.h"
#include "umama_params.h"

std::string umama::get_config_string(Config cfg) {
    switch (cfg) {
        case umama_config::Off:
            return "Off";
        case umama_config::ARM0:
            return "l1_berti=1_l1_nl=0_l1_stream=0_l2_nl=0_l2_stride=0_l2_stream=0_l2_sms=0_l2_bop=0";
        case umama_config::ARM1:
            return "l1_berti=1_l1_nl=0_l1_stream=0_l2_nl=0_l2_stride=0_l2_stream=3_l2_sms=0_l2_bop=0";
        case umama_config::ARM2:
            return "l1_berti=1_l1_nl=0_l1_stream=0_l2_nl=0_l2_stride=0_l2_stream=5_l2_sms=0_l2_bop=0";
        case umama_config::ARM3:
            return "l1_berti=1_l1_nl=0_l1_stream=0_l2_nl=1_l2_stride=0_l2_stream=6_l2_sms=0_l2_bop=0";
        case umama_config::ARM4:
            return "l1_berti=1_l1_nl=0_l1_stream=0_l2_nl=0_l2_stride=3_l2_stream=7_l2_sms=0_l2_bop=0";
        case umama_config::ARM5:
            return "l1_berti=1_l1_nl=0_l1_stream=0_l2_nl=0_l2_stride=0_l2_stream=15_l2_sms=0_l2_bop=0";
        case umama_config::ARM6:
            return "l1_berti=1_l1_nl=0_l1_stream=0_l2_nl=0_l2_stride=3_l2_stream=15_l2_sms=0_l2_bop=0";
        case umama_config::ARM7:
            return "l1_berti=1_l1_nl=0_l1_stream=0_l2_nl=0_l2_stride=5_l2_stream=20_l2_sms=0_l2_bop=0";
        case umama_config::ARM8:
            return "l1_berti=1_l1_nl=0_l1_stream=0_l2_nl=0_l2_stride=5_l2_stream=20_l2_sms=15_l2_bop=0";
        case umama_config::ARM9:
            return "l1_berti=1_l1_nl=0_l1_stream=0_l2_nl=0_l2_stride=0_l2_stream=0_l2_sms=0_l2_bop=1";
        case umama_config::ARM10:
            return "l1_berti=1_l1_nl=0_l1_stream=0_l2_nl=0_l2_stride=0_l2_stream=3_l2_sms=0_l2_bop=1";
        case umama_config::ARM11:
            return "l1_berti=1_l1_nl=0_l1_stream=0_l2_nl=0_l2_stride=0_l2_stream=5_l2_sms=0_l2_bop=1";
        case umama_config::ARM12:
            return "l1_berti=1_l1_nl=0_l1_stream=0_l2_nl=1_l2_stride=0_l2_stream=6_l2_sms=0_l2_bop=1";
        case umama_config::ARM13:
            return "l1_berti=1_l1_nl=0_l1_stream=0_l2_nl=0_l2_stride=3_l2_stream=7_l2_sms=0_l2_bop=1";
        case umama_config::ARM14:
            return "l1_berti=1_l1_nl=0_l1_stream=0_l2_nl=0_l2_stride=0_l2_stream=15_l2_sms=0_l2_bop=1";
        case umama_config::ARM15:
            return "l1_berti=1_l1_nl=0_l1_stream=0_l2_nl=0_l2_stride=3_l2_stream=15_l2_sms=0_l2_bop=1";
        case umama_config::ARM16:
            return "l1_berti=1_l1_nl=0_l1_stream=0_l2_nl=0_l2_stride=5_l2_stream=20_l2_sms=0_l2_bop=1";
        case umama_config::ARM17:
            return "l1_berti=1_l1_nl=0_l1_stream=0_l2_nl=0_l2_stride=5_l2_stream=20_l2_sms=15_l2_bop=1";
        default:
            return "Unknown";
    }
}

uint32_t umama::get_l1_berti_degree(Config cfg) {
    switch (cfg) {
        case umama_config::Off:
            return 1;
        case umama_config::ARM0:
            return 1;
        case umama_config::ARM1:
            return 1;
        case umama_config::ARM2:
            return 1;
        case umama_config::ARM3:
            return 1;
        case umama_config::ARM4:
            return 1;
        case umama_config::ARM5:
            return 1;
        case umama_config::ARM6:
            return 1;
        case umama_config::ARM7:
            return 1;
        case umama_config::ARM8:
            return 1;
        case umama_config::ARM9:
            return 1;
        case umama_config::ARM10:
            return 1;
        case umama_config::ARM11:
            return 1;
        case umama_config::ARM12:
            return 1;
        case umama_config::ARM13:
            return 1;
        case umama_config::ARM14:
            return 1;
        case umama_config::ARM15:
            return 1;
        case umama_config::ARM16:
            return 1;
        case umama_config::ARM17:
            return 1;
        default:
            return 0;
    }
}

uint32_t umama::get_l1_next_line_degree(Config cfg) {
    switch (cfg) {
        case umama_config::Off:
            return 0;
        case umama_config::ARM0:
            return 0;
        case umama_config::ARM1:
            return 0;
        case umama_config::ARM2:
            return 0;
        case umama_config::ARM3:
            return 0;
        case umama_config::ARM4:
            return 0;
        case umama_config::ARM5:
            return 0;
        case umama_config::ARM6:
            return 0;
        case umama_config::ARM7:
            return 0;
        case umama_config::ARM8:
            return 0;
        case umama_config::ARM9:
            return 0;
        case umama_config::ARM10:
            return 0;
        case umama_config::ARM11:
            return 0;
        case umama_config::ARM12:
            return 0;
        case umama_config::ARM13:
            return 0;
        case umama_config::ARM14:
            return 0;
        case umama_config::ARM15:
            return 0;
        case umama_config::ARM16:
            return 0;
        case umama_config::ARM17:
            return 0;
        default:
            return 0;
    }
}

uint32_t umama::get_l1_streamer_degree(Config cfg) {
    switch (cfg) {
        case umama_config::Off:
            return 0;
        case umama_config::ARM0:
            return 0;
        case umama_config::ARM1:
            return 0;
        case umama_config::ARM2:
            return 0;
        case umama_config::ARM3:
            return 0;
        case umama_config::ARM4:
            return 0;
        case umama_config::ARM5:
            return 0;
        case umama_config::ARM6:
            return 0;
        case umama_config::ARM7:
            return 0;
        case umama_config::ARM8:
            return 0;
        case umama_config::ARM9:
            return 0;
        case umama_config::ARM10:
            return 0;
        case umama_config::ARM11:
            return 0;
        case umama_config::ARM12:
            return 0;
        case umama_config::ARM13:
            return 0;
        case umama_config::ARM14:
            return 0;
        case umama_config::ARM15:
            return 0;
        case umama_config::ARM16:
            return 0;
        case umama_config::ARM17:
            return 0;
        default:
            return 0;
    }
}

uint32_t umama::get_next_line_degree(Config cfg) {
    switch (cfg) {
        case umama_config::Off:
            return 0;
        case umama_config::ARM0:
            return 0;
        case umama_config::ARM1:
            return 0;
        case umama_config::ARM2:
            return 0;
        case umama_config::ARM3:
            return 1;
        case umama_config::ARM4:
            return 0;
        case umama_config::ARM5:
            return 0;
        case umama_config::ARM6:
            return 0;
        case umama_config::ARM7:
            return 0;
        case umama_config::ARM8:
            return 0;
        case umama_config::ARM9:
            return 0;
        case umama_config::ARM10:
            return 0;
        case umama_config::ARM11:
            return 0;
        case umama_config::ARM12:
            return 1;
        case umama_config::ARM13:
            return 0;
        case umama_config::ARM14:
            return 0;
        case umama_config::ARM15:
            return 0;
        case umama_config::ARM16:
            return 0;
        case umama_config::ARM17:
            return 0;
        default:
            return 0;
    }
}

uint32_t umama::get_stride_degree(Config cfg) {
    switch (cfg) {
        case umama_config::Off:
            return 0;
        case umama_config::ARM0:
            return 0;
        case umama_config::ARM1:
            return 0;
        case umama_config::ARM2:
            return 0;
        case umama_config::ARM3:
            return 0;
        case umama_config::ARM4:
            return 3;
        case umama_config::ARM5:
            return 0;
        case umama_config::ARM6:
            return 3;
        case umama_config::ARM7:
            return 5;
        case umama_config::ARM8:
            return 5;
        case umama_config::ARM9:
            return 0;
        case umama_config::ARM10:
            return 0;
        case umama_config::ARM11:
            return 0;
        case umama_config::ARM12:
            return 0;
        case umama_config::ARM13:
            return 3;
        case umama_config::ARM14:
            return 0;
        case umama_config::ARM15:
            return 3;
        case umama_config::ARM16:
            return 5;
        case umama_config::ARM17:
            return 5;
        default:
            return 0;
    }
}

uint32_t umama::get_streamer_degree(Config cfg) {
    switch (cfg) {
        case umama_config::Off:
            return 0;
        case umama_config::ARM0:
            return 0;
        case umama_config::ARM1:
            return 3;
        case umama_config::ARM2:
            return 5;
        case umama_config::ARM3:
            return 6;
        case umama_config::ARM4:
            return 7;
        case umama_config::ARM5:
            return 15;
        case umama_config::ARM6:
            return 15;
        case umama_config::ARM7:
            return 20;
        case umama_config::ARM8:
            return 20;
        case umama_config::ARM9:
            return 0;
        case umama_config::ARM10:
            return 3;
        case umama_config::ARM11:
            return 5;
        case umama_config::ARM12:
            return 6;
        case umama_config::ARM13:
            return 7;
        case umama_config::ARM14:
            return 15;
        case umama_config::ARM15:
            return 15;
        case umama_config::ARM16:
            return 20;
        case umama_config::ARM17:
            return 20;
        default:
            return 0;
    }
}

uint32_t umama::get_sms_degree(Config cfg) {
    switch (cfg) {
        case umama_config::Off:
            return 0;
        case umama_config::ARM0:
            return 0;
        case umama_config::ARM1:
            return 0;
        case umama_config::ARM2:
            return 0;
        case umama_config::ARM3:
            return 0;
        case umama_config::ARM4:
            return 0;
        case umama_config::ARM5:
            return 0;
        case umama_config::ARM6:
            return 0;
        case umama_config::ARM7:
            return 0;
        case umama_config::ARM8:
            return 15;
        case umama_config::ARM9:
            return 0;
        case umama_config::ARM10:
            return 0;
        case umama_config::ARM11:
            return 0;
        case umama_config::ARM12:
            return 0;
        case umama_config::ARM13:
            return 0;
        case umama_config::ARM14:
            return 0;
        case umama_config::ARM15:
            return 0;
        case umama_config::ARM16:
            return 0;
        case umama_config::ARM17:
            return 15;
        default:
            return 0;
    }
}

uint32_t umama::get_l2_bop_degree(Config cfg) {
    switch (cfg) {
        case umama_config::Off:
            return 0;
        case umama_config::ARM0:
            return 0;
        case umama_config::ARM1:
            return 0;
        case umama_config::ARM2:
            return 0;
        case umama_config::ARM3:
            return 0;
        case umama_config::ARM4:
            return 0;
        case umama_config::ARM5:
            return 0;
        case umama_config::ARM6:
            return 0;
        case umama_config::ARM7:
            return 0;
        case umama_config::ARM8:
            return 0;
        case umama_config::ARM9:
            return 1;
        case umama_config::ARM10:
            return 1;
        case umama_config::ARM11:
            return 1;
        case umama_config::ARM12:
            return 1;
        case umama_config::ARM13:
            return 1;
        case umama_config::ARM14:
            return 1;
        case umama_config::ARM15:
            return 1;
        case umama_config::ARM16:
            return 1;
        case umama_config::ARM17:
            return 1;
        default:
            return 0;
    }
}

uint32_t umama::get_l3_pythia_degree(Config cfg) {
    switch (cfg) {
        // case umama_config::Off:
        //     return 0;
        // case umama_config::ARM0:
        //     return 1;
        // case umama_config::ARM1:
        //     return 1;
        // case umama_config::ARM2:
        //     return 1;
        // case umama_config::ARM3:
        //     return 1;
        // case umama_config::ARM4:
        //     return 1;
        // case umama_config::ARM5:
        //     return 1;
        // case umama_config::ARM6:
        //     return 1;
        // case umama_config::ARM7:
        //     return 1;
        // case umama_config::ARM8:
        //     return 1;
        // case umama_config::ARM9:
        //     return 1;
        // case umama_config::ARM10:
        //     return 1;
        // case umama_config::ARM11:
        //     return 1;
        // case umama_config::ARM12:
        //     return 1;
        // case umama_config::ARM13:
        //     return 1;
        // case umama_config::ARM14:
        //     return 1;
        // case umama_config::ARM15:
        //     return 1;
        // case umama_config::ARM16:
        //     return 1;
        // case umama_config::ARM17:
        //     return 1;
        default:
            return 0;
    }
}
