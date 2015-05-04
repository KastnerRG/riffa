//
// The following value is arbitrarily chosen from the space defined
// by Microsoft as being "for non-Microsoft use"
//
//
// {40d49fb9-6085-4e1d-8753-822be944d7bb}
DEFINE_GUID (GUID_RIFFA_INTERFACE,
   0x40d49fb9, 0x6085, 0x4e1d, 0x87, 0x53, 0x82, 0x2b, 0xe9, 0x44, 0xd7, 0xbb);

// The IOCTL function codes from 0x800 to 0xFFF are for customer use.
#define CTL_CODE( DeviceType, Function, Method, Access ) (                 \
    ((DeviceType) << 16) | ((Access) << 14) | ((Function) << 2) | (Method) \
)

#define IOCTL_RIFFA_SEND \
    CTL_CODE(0x00000022, 0x900, 2, 0)
#define IOCTL_RIFFA_RECV \
    CTL_CODE(0x00000022, 0x901, 2, 0)
#define IOCTL_RIFFA_LIST \
    CTL_CODE(0x00000022, 0x902, 2 ,0)
#define IOCTL_RIFFA_RESET \
    CTL_CODE(0x00000022, 0x903, 2, 0)

