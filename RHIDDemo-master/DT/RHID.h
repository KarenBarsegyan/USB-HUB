// D:\Devel\RC\Soft\DT\RHID.h


char ReportDescriptor[79] = {
    0x06, 0x00, 0xff,              // USAGE_PAGE (Generic Desktop)
    0x09, 0x01,                    // USAGE (Vendor Usage 1)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x85, 0x01,                    //   REPORT_ID (1)
    0x09, 0x01,                    //   USAGE (Vendor Usage 1)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0xb1, 0x82,                    //   FEATURE (Data,Var,Abs,Vol)
    0x85, 0x01,                    //   REPORT_ID (1)
    0x09, 0x01,                    //   USAGE (Vendor Usage 1)
    0x91, 0x82,                    //   OUTPUT (Data,Var,Abs,Vol)
    0x85, 0x02,                    //   REPORT_ID (2)
    0x09, 0x02,                    //   USAGE (Vendor Usage 2)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0xb1, 0x82,                    //   FEATURE (Data,Var,Abs,Vol)
    0x85, 0x02,                    //   REPORT_ID (2)
    0x09, 0x02,                    //   USAGE (Vendor Usage 2)
    0x91, 0x82,                    //   OUTPUT (Data,Var,Abs,Vol)
    0x85, 0x03,                    //   REPORT_ID (3)
    0x09, 0x03,                    //   USAGE (Vendor Usage 3)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0xb1, 0x82,                    //   FEATURE (Data,Var,Abs,Vol)
    0x85, 0x03,                    //   REPORT_ID (3)
    0x09, 0x03,                    //   USAGE (Vendor Usage 3)
    0x91, 0x82,                    //   OUTPUT (Data,Var,Abs,Vol)
    0x85, 0x04,                    //   REPORT_ID (4)
    0x09, 0x04,                    //   USAGE (Vendor Usage 4)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x02,                    //   REPORT_COUNT (2)
    0x81, 0x82,                    //   INPUT (Data,Var,Abs,Vol)
    0xc0                           // END_COLLECTION
};

