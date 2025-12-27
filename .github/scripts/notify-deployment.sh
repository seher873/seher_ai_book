#!/bin/bash

# Deployment Notification Script
# Sends deployment status notifications to Slack and GitHub
# Usage: ./notify-deployment.sh <event_type> <environment> [version] [commit_sha] [author] [duration] [error_message] [logs_url]

set -e

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
EVENT_TYPE="${1:-}" # started, success, failure, rollback
ENVIRONMENT="${2:-production}"
VERSION="${3:-unknown}"
COMMIT_SHA="${4:-unknown}"
AUTHOR="${5:-unknown}"
DURATION="${6:-}"
ERROR_MESSAGE="${7:-}"
LOGS_URL="${8:-}"

# Validate arguments
if [ -z "$EVENT_TYPE" ]; then
    echo -e "${RED}Error: Event type is required${NC}"
    echo "Usage: $0 <event_type> <environment> [version] [commit_sha] [author] [duration] [error_message] [logs_url]"
    echo "Event types: started, success, failure, rollback"
    exit 1
fi

echo "========================================="
echo "Deployment Notification"
echo "========================================="
echo "Event: $EVENT_TYPE"
echo "Environment: $ENVIRONMENT"
echo "Version: $VERSION"
echo "Commit: $COMMIT_SHA"
echo "========================================="

# Function to get emoji and color based on event type
get_event_styling() {
    case "$EVENT_TYPE" in
        started)
            EMOJI="🚀"
            COLOR="#0066CC"
            TITLE="Deployment Started"
            ;;
        success)
            EMOJI="✅"
            COLOR="#00AA00"
            TITLE="Deployment Succeeded"
            ;;
        failure)
            EMOJI="❌"
            COLOR="#CC0000"
            TITLE="Deployment Failed"
            ;;
        rollback)
            EMOJI="⏪"
            COLOR="#FF8800"
            TITLE="Rollback Initiated"
            ;;
        *)
            EMOJI="ℹ️"
            COLOR="#888888"
            TITLE="Deployment Update"
            ;;
    esac
}

# Function to send Slack notification
send_slack_notification() {
    if [ -z "$SLACK_WEBHOOK_URL" ]; then
        echo -e "${YELLOW}Warning: SLACK_WEBHOOK_URL not set, skipping Slack notification${NC}"
        return 0
    fi

    get_event_styling

    # Build message text
    local text="$EMOJI *$TITLE*"
    if [ -n "$ERROR_MESSAGE" ]; then
        # Escape special characters for JSON
        local escaped_error=$(echo "$ERROR_MESSAGE" | sed 's/"/\\"/g' | head -c 500)
        text="$text\n\`\`\`${escaped_error}\`\`\`"
    fi

    # Build fields array
    local fields="["
    fields+="{\"title\":\"Environment\",\"value\":\"$ENVIRONMENT\",\"short\":true},"
    fields+="{\"title\":\"Version\",\"value\":\"$VERSION\",\"short\":true},"
    fields+="{\"title\":\"Commit\",\"value\":\"${COMMIT_SHA:0:7}\",\"short\":true},"
    fields+="{\"title\":\"Author\",\"value\":\"$AUTHOR\",\"short\":true}"

    if [ -n "$DURATION" ]; then
        fields+=",{\"title\":\"Duration\",\"value\":\"$DURATION\",\"short\":true}"
    fi

    fields+="]"

    # Build actions array if logs URL is provided
    local actions=""
    if [ -n "$LOGS_URL" ]; then
        actions=",\"actions\":[{\"type\":\"button\",\"text\":\"View Logs\",\"url\":\"$LOGS_URL\"}]"
    fi

    # Build payload
    local payload=$(cat <<EOF
{
  "username": "Deployment Bot",
  "icon_emoji": ":rocket:",
  "attachments": [
    {
      "color": "$COLOR",
      "text": "$text",
      "fields": $fields,
      "footer": "Physical AI Textbook CI/CD",
      "ts": $(date +%s)
      $actions
    }
  ]
}
EOF
)

    # Send to Slack
    echo "Sending Slack notification..."
    response=$(curl -s -X POST \
        -H 'Content-Type: application/json' \
        -d "$payload" \
        "$SLACK_WEBHOOK_URL")

    if [ "$response" == "ok" ]; then
        echo -e "${GREEN}✓ Slack notification sent successfully${NC}"
        return 0
    else
        echo -e "${RED}✗ Slack notification failed: $response${NC}"
        return 1
    fi
}

# Function to create GitHub commit status
create_github_status() {
    if [ -z "$GITHUB_TOKEN" ]; then
        echo -e "${YELLOW}Warning: GITHUB_TOKEN not set, skipping GitHub status${NC}"
        return 0
    fi

    if [ "$COMMIT_SHA" == "unknown" ] || [ -z "$COMMIT_SHA" ]; then
        echo -e "${YELLOW}Warning: Commit SHA unknown, skipping GitHub status${NC}"
        return 0
    fi

    # Map event type to GitHub status state
    local state
    case "$EVENT_TYPE" in
        started)
            state="pending"
            ;;
        success)
            state="success"
            ;;
        failure)
            state="failure"
            ;;
        rollback)
            state="error"
            ;;
        *)
            state="pending"
            ;;
    esac

    get_event_styling

    # Build status payload
    local context="deploy-$ENVIRONMENT"
    local description="$TITLE - $ENVIRONMENT"
    if [ -n "$DURATION" ]; then
        description="$description ($DURATION)"
    fi

    # Truncate description if too long (GitHub limit: 140 chars)
    description="${description:0:140}"

    local payload=$(cat <<EOF
{
  "state": "$state",
  "target_url": "${LOGS_URL:-https://github.com}",
  "description": "$description",
  "context": "$context"
}
EOF
)

    # Get repository info from environment
    local repo="${GITHUB_REPOSITORY:-}"
    if [ -z "$repo" ]; then
        echo -e "${YELLOW}Warning: GITHUB_REPOSITORY not set, skipping GitHub status${NC}"
        return 0
    fi

    # Send to GitHub API
    echo "Creating GitHub commit status..."
    response=$(curl -s -X POST \
        -H "Authorization: token $GITHUB_TOKEN" \
        -H "Accept: application/vnd.github.v3+json" \
        -d "$payload" \
        "https://api.github.com/repos/$repo/statuses/$COMMIT_SHA" 2>&1)

    if echo "$response" | grep -q '"id"'; then
        echo -e "${GREEN}✓ GitHub status created successfully${NC}"
        return 0
    else
        echo -e "${RED}✗ GitHub status creation failed${NC}"
        echo "$response"
        return 1
    fi
}

# Function to log deployment event
log_deployment_event() {
    local log_file="deployment-events.json"

    local event=$(cat <<EOF
{
  "timestamp": "$(date -u +"%Y-%m-%dT%H:%M:%SZ")",
  "event_type": "$EVENT_TYPE",
  "environment": "$ENVIRONMENT",
  "version": "$VERSION",
  "commit_sha": "$COMMIT_SHA",
  "author": "$AUTHOR",
  "duration": "$DURATION",
  "error_message": "${ERROR_MESSAGE:0:200}",
  "logs_url": "$LOGS_URL"
}
EOF
)

    echo "$event" >> "$log_file"
    echo "Event logged to $log_file"
}

# Main execution
echo "Executing notification tasks..."

# Send Slack notification (non-blocking)
if send_slack_notification; then
    echo "Slack notification completed"
else
    echo -e "${YELLOW}Slack notification failed (non-blocking)${NC}"
fi

# Create GitHub status (non-blocking)
if create_github_status; then
    echo "GitHub status completed"
else
    echo -e "${YELLOW}GitHub status creation failed (non-blocking)${NC}"
fi

# Log event (always succeeds)
log_deployment_event

# Console output summary
get_event_styling
echo ""
echo "========================================="
echo -e "${BLUE}$EMOJI $TITLE${NC}"
echo "========================================="
echo "Environment: $ENVIRONMENT"
echo "Version: $VERSION"
echo "Commit: ${COMMIT_SHA:0:7}"
if [ -n "$AUTHOR" ] && [ "$AUTHOR" != "unknown" ]; then
    echo "Author: $AUTHOR"
fi
if [ -n "$DURATION" ]; then
    echo "Duration: $DURATION"
fi
if [ -n "$ERROR_MESSAGE" ]; then
    echo -e "${RED}Error: ${ERROR_MESSAGE:0:100}${NC}"
fi
if [ -n "$LOGS_URL" ]; then
    echo "Logs: $LOGS_URL"
fi
echo "========================================="

exit 0
