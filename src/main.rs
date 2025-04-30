// unrar_core.rs - 解凍処理のRust実装コア構造

use std::fs::File;
use std::io::{self, Read};

pub struct CommandData {
    pub command: Vec<char>,
    pub arc_name: String,
    pub extr_path: String,
    pub temp_path: String,
    pub manual_password: bool,
    pub threads: usize,
    pub use_large_pages: bool,
}

pub struct BitInput<'a> {
    buffer: u32,
    bit_count: u8,
    source: &'a [u8],
    pos: usize,
}

impl<'a> BitInput<'a> {
    pub fn new(source: &'a [u8]) -> Self {
        Self {
            buffer: 0,
            bit_count: 0,
            source,
            pos: 0,
        }
    }

    pub fn fgetbits(&mut self) -> u32 {
        while self.bit_count <= 24 && self.pos < self.source.len() {
            self.buffer |= (self.source[self.pos] as u32) << (24 - self.bit_count);
            self.bit_count += 8;
            self.pos += 1;
        }
        self.buffer >> 16
    }

    pub fn faddbits(&mut self, bits: u8) {
        self.buffer <<= bits;
        self.bit_count = self.bit_count.saturating_sub(bits);
    }
}

pub struct Unpack<'a> {
    pub inp: BitInput<'a>,
    pub window: Vec<u8>,
    pub unp_ptr: usize,
    pub wr_ptr: usize,
    pub dest_unp_size: i32,
    pub st_mode: bool,
    pub num_huf: usize,
    pub flags_cnt: u8,
    pub flag_buf: u32,
    pub ch_set: Vec<u16>,
    pub n_to_pl: Vec<u8>,
    pub avr_plc: u32,
    pub nhfb: u32,
    pub nlzb: u32,
}

impl<'a> Unpack<'a> {
    pub fn decode_num(
        &mut self,
        num: u32,
        start_pos: usize,
        dec_tab: &[u32],
        pos_tab: &[u32],
    ) -> u32 {
        let mut num = num & 0xfff0;
        let mut i = 0;
        let mut sp = start_pos;
        while i < dec_tab.len() && dec_tab[i] <= num {
            i += 1;
            sp += 1;
        }
        self.inp.faddbits(sp as u8);
        let dec_base = if i > 0 { dec_tab[i - 1] } else { 0 };
        ((num - dec_base) >> (16 - sp)) + pos_tab[sp]
    }

    pub fn corr_huff(&mut self) {
        for i in (0..=255).rev() {
            let mut j = i;
            let mut _k = 1;
            while j != 0 {
                j >>= 1;
                _k += 1;
            }
            self.ch_set[i] = (i as u16) << 8;
            self.n_to_pl[i] = 0;
        }
    }

    pub fn copy_string15(&mut self, distance: u32, length: u32) {
        for _ in 0..length {
            let src = (self.unp_ptr + self.window.len() - distance as usize) % self.window.len();
            let val = self.window[src];
            self.window[self.unp_ptr] = val;
            self.unp_ptr = (self.unp_ptr + 1) % self.window.len();
        }
    }

    pub fn write_window(&mut self, val: u8) {
        self.window[self.unp_ptr] = val;
        self.unp_ptr += 1;
        self.dest_unp_size -= 1;
    }

    // 修正済み: huff_decode 内の decode_num 呼び出しにおける self.inp の多重借用回避

    pub fn huff_decode(&mut self) {
        let bit_field = self.inp.fgetbits();
        let byte_place = self.decode_num(
            bit_field,
            0,
            &[0x1000, 0x3000, 0x5000, 0x7000, 0xffff],
            &[0, 1, 2, 3, 4, 5],
        );
        let bp = (byte_place & 0xff) as usize;

        if self.st_mode {
            if bp == 0 && bit_field > 0x0fff {
                // 0x100 を u16 として直接出力（特別な値）
                let special = 0x100u16;
                let high = (special >> 8) as u8;
                let low = (special & 0xff) as u8;
                self.write_window(high);
                self.write_window(low);
                return;
            }
            if byte_place == 0 {
                let bit2 = self.inp.fgetbits();
                self.inp.faddbits(1);
                if (bit2 & 0x8000) != 0 {
                    self.st_mode = false;
                    self.num_huf = 0;
                    return;
                } else {
                    let length = if (bit2 & 0x4000) != 0 { 4 } else { 3 };
                    self.inp.faddbits(1);
                    let bits_for_dist = self.inp.fgetbits();
                    let mut distance = self.decode_num(
                        bits_for_dist,
                        2,
                        &[0x2000, 0x4000, 0x6000, 0x8000],
                        &[0, 1, 2, 3, 4],
                    );
                    let bits_for_tail = self.inp.fgetbits();
                    distance = (distance << 5) | (bits_for_tail >> 11);
                    self.inp.faddbits(5);
                    self.copy_string15(distance, length);
                    return;
                }
            }
        } else if self.num_huf >= 16 && self.flags_cnt == 0 {
            self.st_mode = true;
        }

        self.avr_plc = self.avr_plc.wrapping_add(bp as u32);
        self.avr_plc = self.avr_plc.wrapping_sub(self.avr_plc >> 8);
        self.nhfb = self.nhfb.wrapping_add(16);
        if self.nhfb > 0xff {
            self.nhfb = 0x90;
            self.nlzb >>= 1;
        }

        let byte_val = (self.ch_set[bp] >> 8) as u8;
        self.window[self.unp_ptr] = byte_val;
        self.unp_ptr = (self.unp_ptr + 1) % self.window.len();
        self.dest_unp_size -= 1;

        while {
            let cur_byte = self.ch_set[bp];
            let idx = (cur_byte & 0xff) as usize;
            self.n_to_pl[idx] = self.n_to_pl[idx].wrapping_add(1);
            (cur_byte & 0xff) > 0xa1
        } {
            self.corr_huff();
        }

        let new_place = self.n_to_pl[bp] as usize;
        let old = self.ch_set[bp];
        self.ch_set[bp] = self.ch_set[new_place];
        self.ch_set[new_place] = (old & 0xff00) | (bp as u16);
    }
}

pub fn load_rar_segment(path: &str, size: usize) -> io::Result<Vec<u8>> {
    let mut file = File::open(path)?;
    let mut buffer = vec![0u8; size];
    let bytes_read = file.read(&mut buffer)?;
    buffer.truncate(bytes_read);
    Ok(buffer)
}

pub fn unpack_loop(unpack: &mut Unpack) {
    while unpack.dest_unp_size > 0 {
        unpack.huff_decode();
    }
}

pub fn unpack_rar_file(path: &str) -> io::Result<Vec<u8>> {
    let raw = load_rar_segment(path, 1024)?;
    let mut unpack = Unpack {
        inp: BitInput::new(&raw),
        window: vec![0; 4096],
        unp_ptr: 0,
        wr_ptr: 0,
        dest_unp_size: 16, // 仮定義、実際はヘッダ解析で決める
        st_mode: true,
        num_huf: 0,
        flags_cnt: 0,
        flag_buf: 0,
        ch_set: vec![0x4100; 256], // 仮のハフマン表
        n_to_pl: vec![0; 256],
        avr_plc: 0,
        nhfb: 0,
        nlzb: 0,
    };

    unpack_loop(&mut unpack);

    Ok(unpack.window[..unpack.unp_ptr].to_vec())
}

fn main() -> std::io::Result<()> {
    let out = unpack_rar_file("RJ269789.rar")?;
    println!("出力データ: {:?}", out);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bitinput_fgetbits_and_faddbits() {
        let data = vec![0xAA, 0xCC, 0xF0, 0x0F];
        let mut bit_input = BitInput::new(&data);
        let bits = bit_input.fgetbits();
        println!("fgetbits: {:#018b}", bits);
        bit_input.faddbits(4);
        println!(
            "after faddbits(4), fgetbits: {:#018b}",
            bit_input.fgetbits()
        );
    }

    #[test]
    fn test_decode_num_simple() {
        let data = vec![0xFF, 0x00, 0x00, 0x00];
        let bit_input = BitInput::new(&data);
        let dec_tab = vec![0x8000, 0xA000, 0xC000, 0xFFFF];
        let pos_tab = vec![0, 1, 2, 3, 4, 5, 6];
        let mut unpack = Unpack {
            inp: bit_input,
            window: vec![0; 4096],
            unp_ptr: 0,
            wr_ptr: 0,
            dest_unp_size: 0,
            st_mode: false,
            num_huf: 0,
            flags_cnt: 0,
            flag_buf: 0,
            ch_set: vec![0; 256],
            n_to_pl: vec![0; 256],
            avr_plc: 0,
            nhfb: 0,
            nlzb: 0,
        };
        let bits = unpack.inp.fgetbits();
        println!("decode_num input bits: {:#06x}", bits);
        println!(
            "decode_num result: {}",
            unpack.decode_num(bits, 1, &dec_tab, &pos_tab)
        );
    }

    #[test]
    fn test_corr_huff_output() {
        let dummy_input = BitInput::new(&[]);
        let mut unpack = Unpack {
            inp: dummy_input,
            window: vec![0; 1],
            unp_ptr: 0,
            wr_ptr: 0,
            dest_unp_size: 0,
            st_mode: false,
            num_huf: 0,
            flags_cnt: 0,
            flag_buf: 0,
            ch_set: vec![0; 256],
            n_to_pl: vec![0; 256],
            avr_plc: 0,
            nhfb: 0,
            nlzb: 0,
        };
        unpack.corr_huff();
        for i in 0..5 {
            println!(
                "ch_set[{}] = {:#06x}, n_to_pl[{}] = {}",
                i, unpack.ch_set[i], i, unpack.n_to_pl[i]
            );
        }
    }

    #[test]
    fn test_copy_string15_behavior() {
        let dummy_input = BitInput::new(&[]);
        let mut unpack = Unpack {
            inp: dummy_input,
            window: vec![0; 16],
            unp_ptr: 4,
            wr_ptr: 0,
            dest_unp_size: 0,
            st_mode: false,
            num_huf: 0,
            flags_cnt: 0,
            flag_buf: 0,
            ch_set: vec![0; 256],
            n_to_pl: vec![0; 256],
            avr_plc: 0,
            nhfb: 0,
            nlzb: 0,
        };
        unpack.window[0] = 1;
        unpack.window[1] = 2;
        unpack.window[2] = 3;
        unpack.window[3] = 4;
        unpack.copy_string15(4, 3);
        for i in 4..7 {
            println!("window[{}] = {}", i, unpack.window[i]);
        }
    }

    #[test]
    fn test_huff_decode_execution() {
        let dummy_input = BitInput::new(&[0x12, 0x34, 0x56, 0x78]);
        let mut unpack = Unpack {
            inp: dummy_input,
            window: vec![0; 32],
            unp_ptr: 0,
            wr_ptr: 0,
            dest_unp_size: 3,
            st_mode: true,
            num_huf: 0,
            flags_cnt: 0,
            flag_buf: 0,
            ch_set: vec![0x4100; 256],
            n_to_pl: vec![0; 256],
            avr_plc: 0,
            nhfb: 0,
            nlzb: 0,
        };

        unpack_loop(&mut unpack);

        for i in 0..unpack.unp_ptr {
            println!(
                "window[{}] = {} ('{}')",
                i, unpack.window[i], unpack.window[i] as char
            );
        }
    }
}
