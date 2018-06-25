//! NEC Infrared transmission protocol
//! ```
//!      ________________          _   _   _     _   _   _   _   _   _     _     _     _     _   _   _   _   _   _     _   _   _   _   _   _   _     _   _     _     _     _     _     _     _                                                                               ________________       _                       
//! ____|                |________| |_| |_| |___| |_| |_| |_| |_| |_| |___| |___| |___| |___| |_| |_| |_| |_| |_| |___| |_| |_| |_| |_| |_| |_| |___| |_| |___| |___| |___| |___| |___| |___| |_____________________________________________________________________________|                |_____| |______________________
//!     | data code lead          |  0   0     1   0   0   0   0   0     1     1     1     1   0   0   0   0   0     1   0   0   0   0   0   0     1   0     1     1     1     1     1     1 |                                                                               repeat code lead       |       
//!
//!     ________________        _ _ _   _ _ _ _ _ _   _   _   _   _ _ _ _ _ _   _ _ _ _ _ _ _   _ _   _   _   _   _   _   _                                                                             ________________     _                     
//! ____                ________ _ _ ___ _ _ _ _ _ ___ ___ ___ ___ _ _ _ _ _ ___ _ _ _ _ _ _ ___ _ ___ ___ ___ ___ ___ ___ _____________________________________________________________________________                _____ ______________________
//!    | data code lead        | 0 0   1 0 0 0 0 0   1   1   1   1 0 0 0 0 0   1 0 0 0 0 0 0   1 0   1   1   1   1   1   1 |                                                                           |repeat code lead    | |       
//!
//! ```
//!  data code lead = 9ms + 4.5ms
//! repeat code lead = 9ms + 2.25ms
//! 0 = 562.5us + 562.5us = 1.125ms
//! 1 = 562.5us + 1687.5us = 2.25ms
//! DATA: data leading followed by 16 bit address followed by 8 bit data followed by 8 bit inverse of the data = 67.5ms frame
//! REPEAT: repeat leading started 108 ms after the previous leading
//!
#![no_std]
#![deny(unsafe_code)]
#![deny(warnings)]

extern crate nb;

pub struct IrReceiver {
    nec_state: NecState,
    to_usec: u32,
}

impl IrReceiver {
    /// Initiates the state of the NEC protocol receiver
    /// * `to_usec`- Will be used to multiply the 't' given to 'NecReceiver::receive' to get the time in micro seconds.
    pub fn new(to_usec: u32) -> IrReceiver {
        IrReceiver {
            nec_state: NecState::ExpectInactive,
            to_usec: to_usec,
        }
    }

    fn us_between(&self, t: u32, t0: u32) -> u32 {
        (t - t0) * self.to_usec
    }
}

pub enum NecContent {
    /// Valid data frame received: '0xAAAADDNN' where
    /// * AAAA = 16 bit address most likely the upper 8 bit is the inverse of the lower 8 bit to get the timing right
    /// * DD = 8 bit data
    /// * NN = the inverse of the DD data
    Data(u32),

    /// Repeat code received
    Repeat,
}

pub trait NecReceiver {
    //type Result = nb::Result<NecContent, u32>;

    /// This must be called ASAP after the level of the IR receiver changed
    ///
    /// * `t`- time instant (convertable to microsec)
    /// * `active`- level of the IR receiver
    ///
    /// It will move the internal state machine and finally return the received command.
    ///
    /// *Note*: Due to the nonblocking implementation this can be polled arbitrary times
    /// with the correct parameters, not only at IR receiver level changes    
    fn receive(&mut self, t: u32, active: bool) -> nb::Result<NecContent, u32>; //Self::Result;
}

enum NecState {
    ExpectInactive,
    ExpectLeadingActive,
    ExpectLeadingActiveFinish(u32),          //t0
    ExpectLeadingPulseFinish(u32),           //t0
    ExpectDataActiveFinish((u32, u32, u32)), //t0, index, data
    ExpectDataPulseFinish((u32, u32, u32)),  //t0, index, data
}

impl NecReceiver for IrReceiver {
    fn receive(&mut self, t: u32, active: bool) -> nb::Result<NecContent, u32> {
        let tunit = 9000 / 16; //= 9000ms / 16 = 562.5us
        let tol = 1; //= 9000ms/8 timing tolerance

        match self.nec_state {
            NecState::ExpectInactive => if !active {
                //the line is inactive
                self.nec_state = NecState::ExpectLeadingActive;
            },
            NecState::ExpectLeadingActive => if active {
                //leading active pulse started
                self.nec_state = NecState::ExpectLeadingActiveFinish(t);
            },
            NecState::ExpectLeadingActiveFinish(t0) => if !active {
                let dt = self.us_between(t, t0);
                self.nec_state = if (dt >= (16 - tol) * tunit) && (dt <= (16 + tol) * tunit) {
                    //[9000us = 16] leading active pulse ended
                    NecState::ExpectLeadingPulseFinish(t0)
                } else {
                    NecState::ExpectLeadingActive
                };
            },
            NecState::ExpectLeadingPulseFinish(t0) => if active {
                let t_pulse = self.us_between(t, t0);

                if t_pulse <= (16 + 8 + tol) * tunit {
                    if t_pulse < (16 + (4 + 8) / 2) * tunit {
                        //leading signal finished with [2250us = 4] inactive -> 'repeat code' received
                        self.nec_state = NecState::ExpectInactive;
                        return Ok(NecContent::Repeat);
                    } else {
                        //leading signal finished with [4500us = 8] inactive -> address[16], data[8], !data[8] should follow this
                        self.nec_state = NecState::ExpectDataActiveFinish((t, 0, 0));
                    };
                } else {
                    self.nec_state = NecState::ExpectInactive;
                };
            },
            NecState::ExpectDataActiveFinish((t0, index, data)) => if !active {
                let dt = self.us_between(t, t0);

                if dt < (1 + 1) * tunit {
                    //active pulse length is [562us = 1]
                    self.nec_state = NecState::ExpectDataPulseFinish((t0, index, data));
                } else {
                    self.nec_state = NecState::ExpectLeadingActive;
                };
            },
            NecState::ExpectDataPulseFinish((t0, index, data)) => if active {
                let t_pulse = self.us_between(t, t0);

                if t_pulse <= (4 + tol) * tunit {
                    let data = if t_pulse > (2 + 4) * tunit / 2 {
                        //active + inactive pulse length is [2250us = 4]
                        (data << 1) | 1
                    } else {
                        //active + inactive pulse length is [1225us = 2]
                        (data << 1)
                    };

                    if index < 31 {
                        //further bits expected
                        self.nec_state = NecState::ExpectDataActiveFinish((t, index + 1, data));
                    } else {
                        //data receive completed
                        self.nec_state = NecState::ExpectInactive;

                        return if (data ^ 0xFF) & 0xFF == (data >> 8) & 0xFF {
                            //the 4nd byte must be the inverse of 3rd byte
                            Ok(NecContent::Data(data))
                        } else {
                            //'checksum error'
                            Err(nb::Error::Other(data))
                        };
                    };
                } else {
                    self.nec_state = NecState::ExpectInactive;
                };
            },
        };

        Err(nb::Error::WouldBlock)
    }
}

// TODO implement RC5 and other protocols too
