output "webhook_url" {
  description = "Point the Foxglove webhook at this URL, subscribed to recording.imported."
  value       = "${aws_apigatewayv2_stage.default.invoke_url}webhook"
}

output "work_queue_url" {
  value = aws_sqs_queue.work.url
}

output "dlq_url" {
  description = "Inspect this when the DLQ alarm fires."
  value       = aws_sqs_queue.dlq.url
}

output "worker_log_group" {
  value = aws_cloudwatch_log_group.worker.name
}

output "receiver_log_group" {
  description = "Check this first when the health check returns a 500."
  value       = aws_cloudwatch_log_group.receiver.name
}
