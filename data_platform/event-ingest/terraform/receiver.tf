data "archive_file" "receiver" {
  type        = "zip"
  source_dir  = "${path.module}/../build/receiver"
  output_path = "${path.module}/../build/receiver.zip"
}

resource "aws_iam_role" "receiver" {
  name = "${var.name_prefix}-receiver"
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

resource "aws_iam_role_policy_attachment" "receiver_logs" {
  role       = aws_iam_role.receiver.name
  policy_arn = "arn:aws:iam::aws:policy/service-role/AWSLambdaBasicExecutionRole"
}

resource "aws_iam_role_policy" "receiver" {
  name = "${var.name_prefix}-receiver"
  role = aws_iam_role.receiver.id

  policy = jsonencode({
    Version = "2012-10-17"
    Statement = [
      {
        Effect   = "Allow"
        Action   = ["sqs:SendMessage"]
        Resource = aws_sqs_queue.work.arn
      },
      {
        Effect   = "Allow"
        Action   = ["secretsmanager:GetSecretValue"]
        Resource = var.webhook_secret_arn
      },
    ]
  })
}

resource "aws_lambda_function" "receiver" {
  function_name    = "${var.name_prefix}-receiver"
  role             = aws_iam_role.receiver.arn
  handler          = "handler.handler"
  runtime          = "python3.12"
  filename         = data.archive_file.receiver.output_path
  source_code_hash = data.archive_file.receiver.output_base64sha256

  # Verify, enqueue, return. Foxglove calls a response slower than 5s a timeout.
  timeout     = 10
  memory_size = 256
  tags        = var.tags

  environment {
    variables = {
      QUEUE_URL          = aws_sqs_queue.work.url
      WEBHOOK_SECRET_ARN = var.webhook_secret_arn
    }
  }
}

resource "aws_cloudwatch_log_group" "receiver" {
  name              = "/aws/lambda/${aws_lambda_function.receiver.function_name}"
  retention_in_days = 30
  tags              = var.tags
}

resource "aws_apigatewayv2_api" "webhook" {
  name          = "${var.name_prefix}-webhook"
  protocol_type = "HTTP"
  tags          = var.tags
}

resource "aws_apigatewayv2_integration" "receiver" {
  api_id                 = aws_apigatewayv2_api.webhook.id
  integration_type       = "AWS_PROXY"
  integration_uri        = aws_lambda_function.receiver.invoke_arn
  payload_format_version = "2.0"
}

resource "aws_apigatewayv2_route" "webhook" {
  api_id    = aws_apigatewayv2_api.webhook.id
  route_key = "POST /webhook"
  target    = "integrations/${aws_apigatewayv2_integration.receiver.id}"
}

resource "aws_apigatewayv2_stage" "default" {
  api_id      = aws_apigatewayv2_api.webhook.id
  name        = "$default"
  auto_deploy = true
  tags        = var.tags
}

resource "aws_lambda_permission" "apigw" {
  statement_id  = "AllowAPIGatewayInvoke"
  action        = "lambda:InvokeFunction"
  function_name = aws_lambda_function.receiver.function_name
  principal     = "apigateway.amazonaws.com"
  source_arn    = "${aws_apigatewayv2_api.webhook.execution_arn}/*/*"
}
