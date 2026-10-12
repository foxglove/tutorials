data "archive_file" "worker" {
  type        = "zip"
  source_dir  = "${path.module}/../build/worker"
  output_path = "${path.module}/../build/worker.zip"
}

resource "aws_iam_role" "worker" {
  name = "${var.name_prefix}-worker"
  tags = var.tags

  assume_role_policy = jsonencode({
    Version = "2012-10-17"
    Statement = [{
      Effect    = "Allow"
      Principal = { Service = "lambda.amazonaws.com" }
      Action    = "sts:AssumeRole"
    }]
  })
}

resource "aws_iam_role_policy_attachment" "worker_logs" {
  role       = aws_iam_role.worker.name
  policy_arn = "arn:aws:iam::aws:policy/service-role/AWSLambdaBasicExecutionRole"
}

resource "aws_iam_role_policy" "worker" {
  name = "${var.name_prefix}-worker"
  role = aws_iam_role.worker.id

  policy = jsonencode({
    Version = "2012-10-17"
    Statement = [
      {
        Effect = "Allow"
        Action = [
          "sqs:ReceiveMessage",
          "sqs:DeleteMessage",
          "sqs:GetQueueAttributes",
        ]
        Resource = aws_sqs_queue.work.arn
      },
      {
        # Read-only, and only this bucket. Foxglove's own grant is separate.
        Effect = "Allow"
        Action = ["s3:GetObject", "s3:ListBucket"]
        Resource = [
          "arn:aws:s3:::${var.recordings_bucket}",
          "arn:aws:s3:::${var.recordings_bucket}/*",
        ]
      },
      {
        Effect   = "Allow"
        Action   = ["secretsmanager:GetSecretValue"]
        Resource = var.foxglove_api_key_secret_arn
      },
    ]
  })
}

resource "aws_lambda_function" "worker" {
  function_name    = "${var.name_prefix}-worker"
  role             = aws_iam_role.worker.arn
  handler          = "handler.handler"
  runtime          = "python3.12"
  filename         = data.archive_file.worker.output_path
  source_code_hash = data.archive_file.worker.output_base64sha256
  timeout          = var.worker_timeout_seconds
  memory_size      = var.worker_memory_mb
  tags             = var.tags

  environment {
    variables = {
      FOXGLOVE_API                = var.foxglove_api
      FOXGLOVE_API_KEY_SECRET_ARN = var.foxglove_api_key_secret_arn
    }
  }
}

resource "aws_cloudwatch_log_group" "worker" {
  name              = "/aws/lambda/${aws_lambda_function.worker.function_name}"
  retention_in_days = 30
  tags              = var.tags
}

resource "aws_lambda_event_source_mapping" "worker" {
  event_source_arn = aws_sqs_queue.work.arn
  function_name    = aws_lambda_function.worker.arn
  batch_size       = 1

  # Cap concurrency so a backfill does not fan out into the Foxglove API.
  scaling_config {
    maximum_concurrency = 5
  }
}
