use anyhow::Error;
use tokio_util::{bytes::Buf, codec::Decoder};
use tracing::trace;

use crate::{FSpyData, Reading, SceneSettings};

pub struct FSpyDecoder {
    data_length: usize,
    image_length: usize,
    current: Reading,
    data: Option<SceneSettings>,
}
impl Default for FSpyDecoder {
    fn default() -> Self {
        FSpyDecoder {
            data_length: 0,
            image_length: 0,
            current: Reading::Header,
            data: None,
        }
    }
}
impl Decoder for FSpyDecoder {
    type Item = FSpyData;

    type Error = Error;

    fn decode(
        &mut self,
        src: &mut tokio_util::bytes::BytesMut,
    ) -> std::result::Result<Option<Self::Item>, Self::Error> {
        match self.current {
            Reading::Header => {
                if src.len() >= 16 {
                    let package_size: usize = src.copy_to_bytes(4).get_u32_le().try_into().unwrap();
                    let version: usize = src.copy_to_bytes(4).get_u32_le().try_into().unwrap();
                    trace!("package_size {package_size}, version {version}");
                    self.data_length = src.copy_to_bytes(4).get_u32_le().try_into().unwrap();
                    self.image_length = src.copy_to_bytes(4).get_u32_le().try_into().unwrap();
                    trace!(
                        "data length {}, image length {}",
                        self.data_length,
                        self.image_length
                    );
                    self.current = Reading::Data;
                    if src.len() > self.data_length {
                        let data: SceneSettings =
                            serde_json::from_slice(&src.copy_to_bytes(self.data_length))?;
                        self.data = Some(data);
                        self.current = Reading::Image;
                    }
                }
            }
            Reading::Data => todo!(),
            Reading::Image => {
                if src.len() >= self.image_length {
                    let image: Vec<u8> = src.copy_to_bytes(self.image_length).to_vec();
                    return Ok(Some(FSpyData {
                        data: self.data.as_ref().unwrap().clone(),
                        image,
                    }));
                }
            }
        }
        //read header
        Ok(None)
    }
}
