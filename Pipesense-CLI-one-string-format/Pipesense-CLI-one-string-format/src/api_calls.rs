use serde_json::Serializer;
use core::time;
use std::{collections::HashMap, fmt::{Result, format}, hash::Hash, ops::Add, time::SystemTime};
use hex;
#[path = "config.rs"] mod config;
use structopt::StructOpt;
use reqwest::{self, Error, Response};
use chrono::{DateTime, TimeZone, Utc, FixedOffset};

/// A helper function for setting up the body sent in the post request
fn setup_body() -> HashMap<&'static str, String>{
    let mut body: HashMap<&str, String> = HashMap::new();
    body.insert("cmd", "tx".to_string());
    body.insert("port", "2".to_string());
    body.insert("appid", "BE7A15AA".to_string());

    body
}

/// A helper function for sending requests
fn send_request(body: &mut HashMap<&str, String>) -> Vec<core::result::Result<reqwest::blocking::Response, Error>> {
    let args = config::Config::from_args();

    let mut device_arr: Vec<String> = vec![];

    for dev in args.devices{
        match dev.as_str() {
            "1" => device_arr.push("70B3D54996A7C39E".to_string()),
            "2" => device_arr.push("BE7A000000000212".to_string()),
            "3" => device_arr.push("BE7A000000000210".to_string()),
            "4" => device_arr.push("BE7A000000000213".to_string()),
            "5" => device_arr.push("BE7A000000000214".to_string()),
            "6" => device_arr.push("1000000000000004".to_string()),
            "7" => device_arr.push("1000000000000005".to_string()),
            "8" => device_arr.push("1000000000000006".to_string()),
            _ => panic!("Invalid input, Aborting")
        }
    };

    let client = reqwest::blocking::Client::new();
    let mut res_arr: Vec<core::result::Result<reqwest::blocking::Response, Error>> = vec![];
    for deve in device_arr{
        //println!("EUI = {}", &deve);
        body.insert("EUI", deve);
        let res= client.post("https://iotnet.teracom.dk/1/rest")
        .json(&body)
        .header("Authorization", "Bearer vnoVqgAAABFpb3RuZXQudGVyYWNvbS5ka6fsjk89IcmqQwPO2-qRMbs=")
        .send();
        res_arr.push(res);
        println!("{:?}", body);
        body.remove("EUI");
    }
    res_arr
}


/// A function that tells devices to deep sleep for a given amount, time in MS
pub fn deep_sleep(time_in_ms: u32){

    let ans = format!("Sleep {}", time_in_ms);
    let ans_in_hex = hex::encode(&ans);

    //println!("{}", ans.as_str());
    //println!("In function");

    let mut body = setup_body();

    body.insert("data", ans_in_hex.to_owned());
    let res = send_request(&mut body);

    println!("Result from Get Request");
    println!("{:?}", &res);
}

/// This function sets the deepsleep interval after a LoraWAN cycle
pub fn set_deep_sleep_time(time_in_ms: u32){
    let mut body = setup_body();
    let mut local_body: HashMap<&str, &str> = HashMap::new();
    let time_in_ms_string = time_in_ms.to_string();
    local_body.insert("deepsleep_time_in_ms", &time_in_ms_string);
    let local_body_string = serde_json::to_string(&local_body).unwrap();

    let local_body_hex = hex::encode(&local_body_string);
    body.insert("data", local_body_hex.to_owned());

    let res = send_request(&mut body);

    println!("Result from Request");
    println!("{:?}", &res);
}

/// This function forces Lora mode, needs a time to start, and how long the duration should be. 
/// Inputs might be listed as &str types, but it needs to be input in MS
/// start_time is an offset from device wakeup
/// test_dur is how long the duration of the test should be
pub fn goto_lora_mode(start_time: &str, test_dur: &str){
    let mut body = setup_body();
    let mut local_body: HashMap<&str, &str> = HashMap::new();

    local_body.insert("lora_start_time", start_time);
    local_body.insert("lora_test_duration", test_dur);
    let local_body_string = serde_json::to_string(&local_body)
    .unwrap();

    let local_body_hex = hex::encode(&local_body_string);
    body.insert("data", local_body_hex.to_owned());
    let res = send_request(&mut body);

    println!("Result from Get Request");
    println!("{:?}", &res);
}

/// A helper function for setting up the hashmap that will be send.
pub fn ping(){
    let mut body = setup_body();
    let local_body_string = hex::encode("123");
    body.insert("data", local_body_string.to_owned());
    let res = send_request(&mut body);
}

pub fn config_builder(adr: u8, spread_factor: u32, tx_power:u32){
    let mut body = setup_body();
    let mut local_body: HashMap<&str, HashMap<&str, &str>> = HashMap::new();

    local_body.insert("config", HashMap::new());

    let adr_string = adr.to_string();
    let sf_string = spread_factor.to_string();
    let txp_string = tx_power.to_string();

    if adr == 1 || adr == 0{
        local_body.get_mut("config").unwrap().insert("adr", &adr_string);
    }

    if spread_factor != 0 {
        local_body.get_mut("config").unwrap().insert("spread_factor", &sf_string);  
    }

    if tx_power != 0 {
        local_body.get_mut("config").unwrap().insert("tx_power", &txp_string);  
    }

    let local_body_string = serde_json::to_string(&local_body).unwrap();

    let local_body_hex = hex::encode(&local_body_string);
    body.insert("data", local_body_hex.to_owned());

    let res = send_request(&mut body);

    println!("Result from Request");
    println!("{:?}", &res);
}


pub fn set_adr(adr: u8){
    let mut body = setup_body();

    let formatted_string = format!("set_adr:{}", adr);
    let local_body_string = hex::encode(formatted_string);
    body.insert("data", local_body_string.to_owned());
    let res = send_request(&mut body);
}

pub fn set_datarate(datarate: u8){
    let mut body = setup_body();

    let formatted_string = format!("set_datarate:{}", datarate);
    let local_body_string = hex::encode(formatted_string);
    body.insert("data", local_body_string.to_owned());
    let res = send_request(&mut body);
}

/* For new RawLoRa commands */
pub fn rawlora_start_in(starttime: i32){
    let mut body = setup_body();

    let formatted_string = format!("rawlora_start_in:{}", starttime);
    let local_body_string = hex::encode(formatted_string);
    body.insert("data", local_body_string.to_owned());
    let res = send_request(&mut body);
}

pub fn rawlora_duration(duration: i32){
    let mut body = setup_body();

    let formatted_string = format!("rawlora_duration:{}", duration);
    let local_body_string = hex::encode(formatted_string);
    body.insert("data", local_body_string.to_owned());
    let res = send_request(&mut body);
}

pub fn rawlora_periodicity(periodicity: i32){
    let mut body = setup_body();

    let formatted_string = format!("rawlora_periodicity:{}", periodicity);
    let local_body_string = hex::encode(formatted_string);
    body.insert("data", local_body_string.to_owned());
    let res = send_request(&mut body);
}

pub fn rawlora_spreading_factor(SF: i32){
    let mut body = setup_body();

    let formatted_string = format!("rawlora_spreading_factor:{}", SF);
    let local_body_string = hex::encode(formatted_string);
    body.insert("data", local_body_string.to_owned());
    let res = send_request(&mut body);
}

pub fn rawlora_frequency(freq: i32){
    let mut body = setup_body();

    let formatted_string = format!("rawlora_frequency:{}", freq);
    let local_body_string = hex::encode(formatted_string);
    body.insert("data", local_body_string.to_owned());
    let res = send_request(&mut body);
}

pub fn send_command_string(formatted_string: &str){
    println!("Command String: {}", formatted_string); // DEBUG print command string
    let mut body = setup_body();
    let local_body_string = hex::encode(formatted_string);
    body.insert("data", local_body_string.to_owned());
    let res = send_request(&mut body);
}
