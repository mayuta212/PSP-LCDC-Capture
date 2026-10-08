/*
 * Minimal local declarations from PSPSDK pspusbbus.h.
 * Source/reference:
 * https://github.com/pspdev/pspsdk/blob/master/src/usb/pspusbbus.h
 *
 * Kept local because some older PSPSDK installs do not ship this header.
 */
#ifndef OBS16_PSPUSBBUS_LOCAL_H
#define OBS16_PSPUSBBUS_LOCAL_H

#ifdef __cplusplus
extern "C" {
#endif

struct UsbInterface {
    int expect_interface;
    int unk8;
    int num_interface;
};

struct UsbEndpoint {
    int endpnum;
    int unk2;
    int unk3;
};

struct StringDescriptor {
    unsigned char bLength;
    unsigned char bDescriptorType;
    short bString[32];
} __attribute__((packed));

struct DeviceDescriptor {
    unsigned char bLength;
    unsigned char bDescriptorType;
    unsigned short bcdUSB;
    unsigned char bDeviceClass;
    unsigned char bDeviceSubClass;
    unsigned char bDeviceProtocol;
    unsigned char bMaxPacketSize;
    unsigned short idVendor;
    unsigned short idProduct;
    unsigned short bcdDevice;
    unsigned char iManufacturer;
    unsigned char iProduct;
    unsigned char iSerialNumber;
    unsigned char bNumConfigurations;
} __attribute__((packed));

struct ConfigDescriptor {
    unsigned char bLength;
    unsigned char bDescriptorType;
    unsigned short wTotalLength;
    unsigned char bNumInterfaces;
    unsigned char bConfigurationValue;
    unsigned char iConfiguration;
    unsigned char bmAttributes;
    unsigned char bMaxPower;
} __attribute__((packed));

struct InterfaceDescriptor {
    unsigned char bLength;
    unsigned char bDescriptorType;
    unsigned char bInterfaceNumber;
    unsigned char bAlternateSetting;
    unsigned char bNumEndpoints;
    unsigned char bInterfaceClass;
    unsigned char bInterfaceSubClass;
    unsigned char bInterfaceProtocol;
    unsigned char iInterface;
} __attribute__((packed));

struct EndpointDescriptor {
    unsigned char bLength;
    unsigned char bDescriptorType;
    unsigned char bEndpointAddress;
    unsigned char bmAttributes;
    unsigned short wMaxPacketSize;
    unsigned char bInterval;
} __attribute__((packed));

/* PSPSDK: padding is required or the PSP USB hardware can crash. */
struct UsbData {
    unsigned char devdesc[20];

    struct {
        void *pconfdesc;
        void *pinterfaces;
        void *pinterdesc;
        void *pendp;
    } config;

    struct {
        unsigned char desc[12];
        void *pinterfaces;
    } confdesc;

    unsigned char pad1[8];

    struct {
        void *pinterdesc[2];
        unsigned int intcount;
    } interfaces;

    struct {
        unsigned char desc[12];
        void *pendp;
        unsigned char pad[32];
    } interdesc;

    struct {
        unsigned char desc[16];
    } endp[4];
} __attribute__((packed));

struct DeviceRequest {
    unsigned char bmRequestType;
    unsigned char bRequest;
    unsigned short wValue;
    unsigned short wIndex;
    unsigned short wLength;
} __attribute__((packed));

struct UsbDriver {
    const char *name;
    int endpoints;
    struct UsbEndpoint *endp;
    struct UsbInterface *intp;
    void *devp_hi;
    void *confp_hi;
    void *devp;
    void *confp;
    struct StringDescriptor *str;
    int (*recvctl)(int, int, struct DeviceRequest *);
    int (*func28)(int, int, int);
    int (*attach)(int, void *, void *);
    int (*detach)(int, int, int);
    int unk34;
    int (*start_func)(int, void *);
    int (*stop_func)(int, void *);
    struct UsbDriver *link;
};

struct UsbdDeviceReq {
    struct UsbEndpoint *endp;
    void *data;
    int size;
    int unkc;
    void *func;
    int recvsize;
    int retcode;
    int unk1c;
    void *arg;
    void *link;
};

int sceUsbbdRegister(struct UsbDriver *drv);
int sceUsbbdUnregister(struct UsbDriver *drv);
int sceUsbbdReqCancelAll(struct UsbEndpoint *endp);
int sceUsbbdReqSend(struct UsbdDeviceReq *req);

#ifdef __cplusplus
}
#endif
#endif
