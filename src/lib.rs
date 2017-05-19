extern crate libc;

pub struct Linkbot {
    _inner: *mut libc::c_void,
}

impl Linkbot {
    pub fn new(serial_id: &str) -> Linkbot {
        let _serial_id = serial_id.to_string();
        let _serial_id = _serial_id + "\0";
        unsafe {
            let inner = linkbotFromSerialId((&_serial_id).as_ptr());
            Linkbot{ _inner: inner }
        }
    }

    pub fn move_motors(&mut self, mask: u32, j1: f64, j2: f64, j3: f64) {
        unsafe{
            linkbotMove(self._inner, mask, j1, j2, j3);
        }
    }
}

#[link(name = "linkbot")]
extern {
    fn linkbotFromSerialId(serial_id: *const u8) -> *mut libc::c_void;
    fn linkbotMove(linkbot_inner: *mut libc::c_void, 
                   mask: libc::c_uint,
                   j1: libc::c_double,
                   j2: libc::c_double,
                   j3: libc::c_double) -> libc::c_int;
}

#[cfg(test)]
mod tests {
    use super::Linkbot;
    #[test]
    fn it_works() {
        let mut l = Linkbot::new("ZRG6");
        l.move_motors(7, 90.0, 90.0, 90.0);
    }
}
