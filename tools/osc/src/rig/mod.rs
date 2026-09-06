//! Shared rig plumbing for the experiment subcommands: the bus connection,
//! the four-arm driver pump, table snapshot/rollback, CSV record/replay, and
//! the TEL side-channel. `ident` drives these; `cal` reaches the same set.

pub(crate) mod csvio;
pub(crate) mod pump;
pub(crate) mod snapshot;
pub(crate) mod tel;

use anyhow::{Context, Result, bail};
use osc_client::blocking::Client;
use osc_client::nusb::NusbPipe;

pub(crate) fn connect(baud: &str) -> Result<Client<NusbPipe>> {
    let mut c = Client::connect(NusbPipe::open()?)?;
    match baud {
        "auto" => match c.find_bus_baud()? {
            Some(rate) => println!("bus at {} baud", rate.as_hz()),
            None => bail!("no servo bus found at any supported baud"),
        },
        s => {
            use osc_client::BaudRate as B;
            let bps: u32 = s.parse().context("baud is an integer rate or `auto`")?;
            let rate = [B::B500000, B::B1000000, B::B2000000, B::B3000000]
                .into_iter()
                .find(|r| r.as_hz() == bps)
                .ok_or_else(|| anyhow::anyhow!("unsupported baud {bps}"))?;
            c.host_baud(rate)?;
        }
    }
    Ok(c)
}
