let led = UInt32(PICO_DEFAULT_LED_PIN)
stdio_init_all()
gpio_init(led)
gpio_set_dir(led, /*out*/true)
while true {
  gpio_put(led, true)
  sleep_ms(250)
  gpio_put(led, false)
  sleep_ms(250)
  print("Hello from Pico")
}
