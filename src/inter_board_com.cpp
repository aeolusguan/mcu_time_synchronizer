#include "mcu_time_synchronizer/inter_board_com.h"

curi::USB_COM::USB_COM(int in_length_8b,
                       int out_length_8b,
                       Vendor_id_Hex _vendor_id,
                       Product_id_Hex _product_id)
    : usb_message_in_length(in_length_8b),
      usb_message_out_length(out_length_8b),
      Vendor_id(_vendor_id),
      Product_id(_product_id) {
  usb_message_in_checklength = usb_message_in_length / 4 - 1;
  usb_message_out_checklength = usb_message_out_length / 4 - 1;
  usb_message_32_2_8 = usb_message_out_length / 4;
  usb_message_8_2_32 = usb_message_in_length / 4;
  rx_buff = new uint8_t[usb_message_in_length];
  tx_buff = new uint8_t[usb_message_out_length];
  transfer_tx = libusb_alloc_transfer(0);
  transfer_rx = libusb_alloc_transfer(0);

  libusb_init(&ctx);

  deviceHandle = libusb_open_device_with_vid_pid(ctx,
                                                 Vendor_id.vendor_id,
                                                 Product_id.product_id);
  if (libusb_kernel_driver_active(deviceHandle, 0x00)) {
    int success = libusb_detach_kernel_driver(deviceHandle, 0x00);
    if (success != 0) {
      std::cerr << "Detach Driver Failed!" << std::endl;
      std::abort();
    }
  }
  int claim_interface = libusb_claim_interface(deviceHandle, 0x00);
  if (claim_interface != 0) {
    std::cerr << "Claim Driver Failed!" << std::endl;
    std::abort();
  }
  std::cerr << "[MOTOR USB]: INITIALIZATION SUCCESS!" << std::endl;

  if (pthread_mutex_init(&usb_com_rx_mutex, nullptr) != 0)
    std::cerr << "[ERROR: RT USB] Failed to create usb rx mutex\n";
  if (pthread_mutex_init(&usb_com_tx_mutex, nullptr) != 0)
    std::cerr << "[ERROR: RT USB] Failed to create usb tx mutex\n";
}

curi::USB_COM::~USB_COM() {
  try {
    delete[] tx_buff;
    delete[] rx_buff;
    std::cerr << "[Release devices]\n";
    libusb_free_transfer(transfer_tx);
    libusb_free_transfer(transfer_rx);
    libusb_release_interface(deviceHandle, 0);
    libusb_close(deviceHandle);
    libusb_exit(nullptr);
  }
  catch (std::exception &e) {
    std::cerr << "[Failed] Cannot destroy constructor!\n";
    std::abort();
  }
}

void curi::USB_COM::lock_mutex_tx() {
  pthread_mutex_lock(&usb_com_tx_mutex);
}

void curi::USB_COM::unlock_mutex_tx() {
  pthread_mutex_unlock(&usb_com_tx_mutex);
}

void curi::USB_COM::lock_mutex_rx() {
  pthread_mutex_lock(&usb_com_rx_mutex);
}

void curi::USB_COM::unlock_mutex_rx() {
  pthread_mutex_unlock(&usb_com_rx_mutex);
}

libusb_context *&curi::USB_COM::get_usb_ctx() {
  return ctx;
}

libusb_device_handle *&curi::USB_COM::get_device_handle() {
  return deviceHandle;
}