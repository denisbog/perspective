use anyhow::Error;
use tokio_util::{bytes::BytesMut, codec::Encoder};

use crate::FSpyData;

#[derive(Default)]
pub struct FSpyEncoder {}

impl Encoder<FSpyData> for FSpyEncoder {
    type Error = Error;

    fn encode(
        &mut self,
        item: FSpyData,
        dst: &mut BytesMut,
    ) -> std::result::Result<(), Self::Error> {
        let magic: u32 = 2037412710;
        let version: u32 = 1;
        dst.extend_from_slice(&magic.to_le_bytes());
        dst.extend_from_slice(&version.to_le_bytes());
        let data_string = serde_json::to_string(&item.data).unwrap();
        let data = data_string.as_bytes();
        dst.extend_from_slice(&u32::try_from(data.len())?.to_le_bytes());
        dst.extend_from_slice(&u32::try_from(item.image.len())?.to_le_bytes());
        dst.extend_from_slice(data);
        dst.extend_from_slice(&item.image);
        Ok(())
    }
}
