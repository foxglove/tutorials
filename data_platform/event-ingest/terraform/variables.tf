variable "region" {
  description = "AWS region. Use the same region as the recordings bucket."
  type        = string
}

variable "name_prefix" {
  description = "Prefix for created resource names."
  type        = string
  default     = "foxglove-event-ingest"
}

variable "recordings_bucket" {
  description = "BYOS bucket holding the recordings. The worker is granted read-only access to it."
  type        = string

  validation {
    condition     = var.recordings_bucket != "your-byos-bucket"
    error_message = "Set recordings_bucket in terraform.tfvars to your BYOS bucket."
  }
}

variable "foxglove_api_key_secret_arn" {
  description = <<-EOT
    ARN of an existing Secrets Manager secret holding the Foxglove API key.
    Created outside Terraform on purpose: a secret passed as a variable ends up
    in state in plaintext.
  EOT
  type        = string

  validation {
    condition     = !strcontains(var.foxglove_api_key_secret_arn, "111122223333")
    error_message = "Set foxglove_api_key_secret_arn in terraform.tfvars. The example value is a placeholder."
  }
}

variable "webhook_secret_arn" {
  description = "ARN of an existing Secrets Manager secret holding the Foxglove webhook signing token."
  type        = string

  validation {
    condition     = !strcontains(var.webhook_secret_arn, "111122223333")
    error_message = "Set webhook_secret_arn in terraform.tfvars. The example value is a placeholder."
  }
}

variable "foxglove_api" {
  description = "Foxglove API base URL."
  type        = string
  default     = "https://api.foxglove.dev/v1"
}

variable "worker_timeout_seconds" {
  description = "Worker timeout. Must be well under the queue visibility timeout."
  type        = number
  default     = 120
}

variable "worker_memory_mb" {
  description = "Worker memory. Range reads keep this small; raise only if recordings are large."
  type        = number
  default     = 512
}

variable "max_receive_count" {
  description = "Deliveries attempted before a message goes to the dead-letter queue."
  type        = number
  default     = 3
}

variable "tags" {
  description = "Tags applied to every resource."
  type        = map(string)
  default     = {}
}
