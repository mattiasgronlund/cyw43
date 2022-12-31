#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(async_fn_in_trait)]
#![allow(incomplete_features)]

mod vsys_cyw43;
use cyw43::Control;
use defmt::*;
use embassy_executor::Spawner;
use embassy_net::tcp::TcpSocket;
use embassy_net::{Stack, StackResources};
use embassy_rp::adc::Adc;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::interrupt;
use embassy_rp::peripherals::PIN_23;
use embassy_time::{Duration, Timer};
use embedded_io::asynch::Write;
use static_cell::StaticCell;
use vsys_cyw43::VSys;
use {defmt_rtt as _, panic_probe as _};

macro_rules! singleton {
    ($val:expr) => {{
        type T = impl Sized;
        static STATIC_CELL: StaticCell<T> = StaticCell::new();
        STATIC_CELL.init_with(move || $val)
    }};
}

#[embassy_executor::task]
async fn wifi_task(runner: cyw43::Runner<'static, Output<'static, PIN_23>, vsys_cyw43::Cyw43Spi<'static>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<cyw43::NetDriver<'static>>) -> ! {
    stack.run().await
}

#[embassy_executor::task]
async fn status_task(mut control: Control<'static>, vsys: VSys<'static>, mut adc: Adc<'static>) -> ! {
    let mut led = true;
    loop {
        control.gpio_set(0, led).await;
        led = !led;
        control.gpio_set(1, true).await;
        let vsys_volt = vsys.read_vsys_voltage(&mut adc).await;
        info!("VSys: {} volt", vsys_volt);
        control.gpio_set(1, false).await;
        Timer::after(Duration::from_millis(2000)).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Hello World!");

    let p = embassy_rp::init(Default::default());

    // Include the WiFi firmware and Country Locale Matrix (CLM) blobs.
    let fw = include_bytes!("../../../firmware/43439A0.bin");
    let clm = include_bytes!("../../../firmware/43439A0_clm.bin");

    // To make flashing faster for development, you may want to flash the firmwares independently
    // at hardcoded addresses, instead of baking them into the program with `include_bytes!`:
    // probe-rs-cli download ../../firmware/43439A0.bin --format bin --chip RP2040 --base-address 0x10100000
    // probe-rs-cli download ../../firmware/43439A0_clm.bin --format bin --chip RP2040 --base-address 0x10140000
    // let fw = unsafe { core::slice::from_raw_parts(0x10100000 as *const u8, 224190) };
    // let clm = unsafe { core::slice::from_raw_parts(0x10140000 as *const u8, 4752) };

    let pwr = Output::new(p.PIN_23, Level::Low);

    // let cs = Output::new(p.PIN_25, Level::High);
    // let clk = Output::new(p.PIN_29, Level::Low);
    // let mut dio = Flex::new(p.PIN_24);
    // dio.set_low();
    // dio.set_as_output();

    // let bus = MySpi { clk, dio };
    // let spi = ExclusiveDevice::new(bus, cs);

    let vsys_cyw43_state = singleton!(vsys_cyw43::State::new(p.PIN_24, p.PIN_25, p.PIN_29));
    let (vsys, spi) = vsys_cyw43::new(vsys_cyw43_state);

    let state = singleton!(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;

    spawner.spawn(wifi_task(runner)).unwrap();

    control.init(clm).await;

    //control.join_open(env!("WIFI_NETWORK")).await;
    control.join_wpa2(env!("WIFI_NETWORK"), env!("WIFI_PASSWORD")).await;

    let adc_irq = interrupt::take!(ADC_IRQ_FIFO);
    let adc = Adc::new(p.ADC, adc_irq, embassy_rp::adc::Config::default());

    spawner.spawn(status_task(control, vsys, adc)).unwrap();

    let config = embassy_net::ConfigStrategy::Dhcp;
    //let config = embassy_net::ConfigStrategy::Static(embassy_net::Config {
    //    address: Ipv4Cidr::new(Ipv4Address::new(192, 168, 69, 2), 24),
    //    dns_servers: Vec::new(),
    //    gateway: Some(Ipv4Address::new(192, 168, 69, 1)),
    //});

    // Generate random seed
    let seed = 0x0123_4567_89ab_cdef; // chosen by fair dice roll. guarenteed to be random.

    // Init network stack
    let stack = &*singleton!(Stack::new(
        net_device,
        config,
        singleton!(StackResources::<1, 2, 8>::new()),
        seed
    ));

    unwrap!(spawner.spawn(net_task(stack)));

    // And now we can use it!

    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];
    let mut buf = [0; 4096];

    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(embassy_net::SmolDuration::from_secs(10)));

        info!("Listening on TCP:1234...");
        if let Err(e) = socket.accept(1234).await {
            warn!("accept error: {:?}", e);
            continue;
        }

        info!("Received connection from {:?}", socket.remote_endpoint());

        loop {
            let n = match socket.read(&mut buf).await {
                Ok(0) => {
                    warn!("read EOF");
                    break;
                }
                Ok(n) => n,
                Err(e) => {
                    warn!("read error: {:?}", e);
                    break;
                }
            };

            info!("rxd {:02x}", &buf[..n]);

            match socket.write_all(&buf[..n]).await {
                Ok(()) => {}
                Err(e) => {
                    warn!("write error: {:?}", e);
                    break;
                }
            };
        }
    }
}
