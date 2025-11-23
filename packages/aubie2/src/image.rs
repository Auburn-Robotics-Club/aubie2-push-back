use vexide::{math::Point2, prelude::Display};

pub struct Image<const WIDTH: u32, const HEIGHT: u32> {
    buf: Box<[u32]>,
}

impl<const WIDTH: u32, const HEIGHT: u32> Image<WIDTH, HEIGHT> {
    pub fn decode_png(ibuf: &[u8]) -> Self {
        unsafe {
            let mut data = vec![0; (WIDTH * HEIGHT * 4) as usize].into_boxed_slice();
            let mut img: vex_sdk::v5_image = vex_sdk::v5_image {
                width: WIDTH as _,
                height: HEIGHT as _,
                data: data.as_mut_ptr(),
                p: core::ptr::null_mut(),
            };

            if vex_sdk::vexImagePngRead(ibuf.as_ptr(), &mut img, WIDTH, HEIGHT, ibuf.len() as _)
                != 1
            {
                panic!("Failed to decode PNG image.");
            }

            Image { buf: data }
        }
    }

    pub fn draw(&self, _: &mut Display, offset: impl Into<Point2<i16>>) {
        let offset = offset.into();

        let x2 = offset.x + WIDTH as i16 - 1;
        let y2 = offset.y + HEIGHT as i16 - 1;

        unsafe {
            vex_sdk::vexDisplayCopyRect(
                offset.x as _,
                (offset.y + Display::HEADER_HEIGHT) as i32,
                x2 as _,
                (y2 + Display::HEADER_HEIGHT) as _,
                self.buf.as_ptr().cast_mut(),
                WIDTH as _,
            );
        }
    }
}
