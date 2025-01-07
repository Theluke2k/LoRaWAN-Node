use structopt::StructOpt;
#[derive(StructOpt, Debug)]
#[structopt(name = "config")]

/// This is seup for StructOpt package, that takes the input from the console.
pub struct Config{

    ///Sleep the device for the given time interval, in Miliseconds
    #[structopt(short,long, value_name = "TIME IN MS", default_value = "0")]
    pub uplink_periodicity: u32,

    ///Sleep the device with no way of waking it up remotly - USE WITH CAUTION!!
    #[structopt(long)]
    pub deep_sleep_no_timer_i_know_what_i_am_doing: bool,

    ///Sets an offset, from lates wake up, till when the device has to go into LoraRAW mode
    #[structopt(long, default_value = "", value_name = "TIME IN MS")]
    pub lora_start_time: String, 

    ///Sets how long the following LoraRAW test should be
    #[structopt(long, default_value = "", value_name = "TIME IN MS")]
    pub lora_test_duration: String,

    ///Sets for how long the deepsleep interval between wakeups in LoraWAN mode should be
    #[structopt(long, default_value = "0", value_name = "TIME IN MS")]
    pub deepsleep_time_in_ms: u32,

    /// Set wether or not ADR should be enabled (Note that if this flag is not present, ADR will be set to false)
    #[structopt(long, default_value = "2")]
    pub set_adr: u8,

    /// Set wether or not ADR should be enabled (Note that if this flag is not present, ADR will be set to false)
    #[structopt(long, default_value = "10")]
    pub set_datarate: u8,

    ///Set the spread-factor
    #[structopt(long, default_value = "0")]
    pub set_spread_factor: u32,

    ///Set the TX power
    #[structopt(long, default_value = "0")]
    pub set_tx_power: u32,

    ///Ping the devices. (Simply sends the string "123")
    #[structopt(long)]
    pub ping: bool,

    #[structopt(long)]
    pub devices: Vec<String>,

    /*
    * Lucas (23-08-2024): Commands for RawLoRa.
    */
    #[structopt(long, value_name = "TIME IN MS", default_value = "-1")]
    pub rawlora_start_in: i32,

    #[structopt(long, value_name = "TIME IN MS", default_value = "-1")]
    pub rawlora_duration: i32,

    #[structopt(long, value_name = "TIME IN MS", default_value = "-1")]
    pub rawlora_periodicity: i32,

    #[structopt(long, value_name = "TIME IN MS", default_value = "-1")]
    pub rawlora_spreading_factor: i32,

    #[structopt(long, value_name = "TIME IN MS", default_value = "-1")]
    pub rawlora_frequency: i32

    
}