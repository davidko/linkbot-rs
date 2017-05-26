extern crate libc;
use std::sync::mpsc::{Sender, Receiver};
use std::sync::{Mutex, Condvar};


pub struct Linkbot {
    inner: Box<Inner>
}

#[repr(C)]
pub struct Inner {
    c_impl: *mut libc::c_void,
    movewait_mask: (Mutex<u8>, Condvar),
    motor_mask: u8
    //_movewait_channel: Sender<i32, i32>,
}

extern "C" fn linkbot_joint_event_callback(joint_no: i32, event: i32, timestamp: i32, linkbot: *mut Inner)
{
    unsafe {
        let &(ref lock, ref cond) = &((*linkbot).movewait_mask);

        let mut mask = lock.lock().unwrap();
        if (event == 0) || (event == 1) {
            *mask &= !(1<<joint_no);
            cond.notify_all();
        }
        //let mask = (*linkbot).movewait_mask.lock().unwrap();
    }
}

impl Linkbot {
    pub fn new(serial_id: &str) -> Option<Linkbot> {
        let _serial_id = serial_id.to_string();
        let _serial_id = _serial_id + "\0";
        unsafe {
            let inner = linkbotFromSerialId((&_serial_id).as_ptr());
            if inner.is_null() {
                None
            } else {
                // Enable joint events
                let linkbot = Inner{c_impl: inner,
                                    movewait_mask: (Mutex::new(0), Condvar::new()),
                                    motor_mask: 0,
                };
                let mut linkbot_boxed = Box::new(linkbot);
                linkbotSetJointEventCallback(linkbot_boxed.c_impl, 
                                             linkbot_joint_event_callback,
                                             &mut *linkbot_boxed);
                let mut form:u32 = 0;
                linkbotGetFormFactor(linkbot_boxed.c_impl, &mut form);
                match form {
                    0 => linkbot_boxed.motor_mask = 0x05,
                    1 => linkbot_boxed.motor_mask = 0x03,
                    3 => linkbot_boxed.motor_mask = 0x07,
                    _ => linkbot_boxed.motor_mask = 0,
                }

                Some(Linkbot{ inner: linkbot_boxed })
            }
        }
    }

    pub fn move_motors(&mut self, mask: u8, j1: f64, j2: f64, j3: f64) {
        let &(ref lock, _) = &(self.inner.movewait_mask);
        {
            let mut _mask = lock.lock().unwrap();
            *_mask |= mask & self.inner.motor_mask;
        }
        unsafe{
            linkbotMove(self.inner.c_impl, mask as u32, j1, j2, j3);
        }
    }

    pub fn move_wait(&mut self, mask: u8) {
        let &(ref lock, ref cond) = &(self.inner.movewait_mask);
        let mut _mask = lock.lock().unwrap();
        while (mask & *_mask) != 0 {
            _mask = cond.wait(_mask).unwrap();
        }
    }
}

#[link(name = "linkbot")]
extern {
    fn linkbotFromSerialId(serial_id: *const u8) -> *mut libc::c_void;
    fn linkbotGetFormFactor(linkbot_inner: *mut libc::c_void,
                            form: *mut libc::c_uint);
    fn linkbotMove(linkbot_inner: *mut libc::c_void, 
                   mask: libc::c_uint,
                   j1: libc::c_double,
                   j2: libc::c_double,
                   j3: libc::c_double) -> libc::c_int;
    fn linkbotSetJointEventCallback(linkbot_inner: *mut libc::c_void,
                                    cb: extern fn(i32, i32, i32, *mut Inner),
                                    userdata: *mut Inner);
}

#[cfg(test)]
mod tests {
    use super::Linkbot;
    #[test]
    fn it_works() {
        let mut l = Linkbot::new("ZRG6").unwrap();
        l.move_motors(7, 90.0, 90.0, 90.0);
        l.move_wait(7);
    }
}
