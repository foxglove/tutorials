// Rust imports
use std::sync::{Arc, Mutex};
// rclrs related imports
use rclrs::{Publisher, RclrsError};
use std_msgs::msg::String as StringMsg;
use std_msgs::msg::UInt8 as UInt8Msg;

// Define the struct for the node
struct StringLengthNode {
    node: Arc<rclrs::Node>,
    _subscription: Arc<rclrs::Subscription<StringMsg>>,
    buffer: Arc<Mutex<Vec<Option<StringMsg>>>>,
    publisher: Arc<rclrs::Publisher<UInt8Msg>>,
}

// Define the implementation of the node.
impl StringLengthNode {
    // This function is called when creating the node.
    fn new(context: &rclrs::Context) -> Result<Self, rclrs::RclrsError> {
        let node = rclrs::Node::new(context, "string_length_node")?;
        let buffer = Arc::new(Mutex::new(Vec::new()));
        let buffer_cb = buffer.clone();
        let _subscription = node.create_subscription(
            "string_topic",
            rclrs::QOS_PROFILE_DEFAULT,
            move |msg: StringMsg| {
                // Add the new message to a buffer
                buffer_cb.lock().unwrap().push(Some(msg.clone()));
            },
        )?;
        let publisher = node.create_publisher("string_length", rclrs::QOS_PROFILE_DEFAULT)?;
        // Return Ok with the constructed node
        Ok(Self {
            node,
            _subscription,
            publisher,
            buffer,
        })
    }

    // This function is called when publishing
    fn publish(&self) -> Result<(), rclrs::RclrsError> {
        // Get the latest data from the subscription buffer
        if let Some(s) = self.buffer.lock().unwrap().pop() {
            let mut length_msg = UInt8Msg { data: 0 };
            length_msg.data = s.unwrap().data.len() as u8;
            self.publisher.publish(length_msg)?;
        }
        Ok(())
    }
}

fn main() -> Result<(), rclrs::RclrsError> {
    println!("Hello, world! - String length node.");
    // Create the rclrs context.
    let context = rclrs::Context::new(std::env::args())?;
    // Create a node and a clone. The first one will subscribe and the clone will publish
    let string_length_node = Arc::new(StringLengthNode::new(&context)?);
    let string_length_publish_node = Arc::clone(&string_length_node);
    // Thread for timer to publish
    std::thread::spawn(move || -> Result<(), rclrs::RclrsError> {
        loop {
            use std::time::Duration;
            std::thread::sleep(Duration::from_millis(1000));
            string_length_publish_node.publish()?;
        }
    });
    // Spin the subscription node
    rclrs::spin(Arc::clone(&string_length_node.node))
}
