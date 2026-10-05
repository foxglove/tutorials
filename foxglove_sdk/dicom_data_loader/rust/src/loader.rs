use foxglove::schemas::{FrameTransforms, ImageAnnotations, PointCloud, RawImage, SceneUpdate};
use foxglove_data_loader::{
    BackfillArgs, DataLoader, DataLoaderArgs, Initialization, InitializationBuilder, Message,
    MessageIterator, MessageIteratorArgs, Problem, console, reader,
};

use crate::messages::{MetadataMsg, PhaseStatsMsg, SliceStatsMsg};
use crate::study::{EmptyStudy, MessageIter, SchemaKind, Study, load_study};

pub struct DicomLoader {
    paths: Vec<String>,
    study: Option<Study>,
}

impl DataLoader for DicomLoader {
    type MessageIterator = HostIter;
    type Error = anyhow::Error;

    fn new(args: DataLoaderArgs) -> Self {
        Self {
            paths: args.paths,
            study: None,
        }
    }

    fn initialize(&mut self) -> Result<Initialization, Self::Error> {
        match load_study(&self.paths, |path| Ok(reader::open(path))) {
            Ok(study) => {
                console::log(study.log_line());
                let init = describe(&study)?;
                self.study = Some(study);
                Ok(init)
            }
            Err(empty) => Ok(empty_init(empty)),
        }
    }

    fn create_iter(
        &mut self,
        args: MessageIteratorArgs,
    ) -> Result<Self::MessageIterator, Self::Error> {
        let inner = self
            .study
            .as_ref()
            .map(|study| study.messages(&args.channels, args.start_time, args.end_time));
        Ok(HostIter { inner })
    }

    fn get_backfill(&mut self, args: BackfillArgs) -> Result<Vec<Message>, Self::Error> {
        Ok(match &self.study {
            Some(study) => study.backfill(args.time, &args.channels),
            None => Vec::new(),
        })
    }
}

pub struct HostIter {
    inner: Option<MessageIter>,
}

impl MessageIterator for HostIter {
    type Error = anyhow::Error;

    fn next(&mut self) -> Option<Result<Message, Self::Error>> {
        self.inner.as_mut()?.next()
    }
}

fn describe(study: &Study) -> anyhow::Result<Initialization> {
    let (start, end) = study.time_range();
    let mut builder = Initialization::builder().start_time(start).end_time(end);
    for warning in study.warnings() {
        builder = builder.add_problem(Problem::warn(warning.as_str()));
    }
    add_schema::<RawImage>(&mut builder, study, SchemaKind::RawImage)?;
    add_schema::<ImageAnnotations>(&mut builder, study, SchemaKind::ImageAnnotations)?;
    add_schema::<PointCloud>(&mut builder, study, SchemaKind::PointCloud)?;
    add_schema::<SceneUpdate>(&mut builder, study, SchemaKind::SceneUpdate)?;
    add_schema::<FrameTransforms>(&mut builder, study, SchemaKind::FrameTransforms)?;
    add_schema::<SliceStatsMsg>(&mut builder, study, SchemaKind::SliceStats)?;
    add_schema::<PhaseStatsMsg>(&mut builder, study, SchemaKind::PhaseStats)?;
    add_schema::<MetadataMsg>(&mut builder, study, SchemaKind::Metadata)?;
    Ok(builder.build())
}

fn add_schema<T: foxglove::Encode>(
    builder: &mut InitializationBuilder,
    study: &Study,
    kind: SchemaKind,
) -> anyhow::Result<()> {
    let relevant: Vec<_> = study
        .channels()
        .iter()
        .filter(|channel| channel.kind == kind)
        .collect();
    if relevant.is_empty() {
        return Ok(());
    }
    let schema = builder.add_encode::<T>()?;
    for channel in relevant {
        schema
            .add_channel_with_id(channel.id, channel.topic)
            .expect("channel id is unique")
            .message_count(channel.message_count);
    }
    Ok(())
}

fn empty_init(empty: EmptyStudy) -> Initialization {
    let mut builder =
        Initialization::builder().add_problem(Problem::error(empty.message).tip(empty.tip));
    for warning in empty.warnings {
        builder = builder.add_problem(Problem::warn(warning));
    }
    builder.build()
}
