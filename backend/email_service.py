import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from typing import Any
from .config import settings


def send_verification_email(email: str, name: str, token: str) -> Any:
    """
    Send verification email to the user
    """
    try:
        msg = MIMEMultipart()
        msg['From'] = settings.MAIL_FROM
        msg['To'] = email
        msg['Subject'] = "Verify your email address"

        # Create the HTML body of the email
        html = f"""
        <html>
          <body>
            <h2>Welcome to our platform, {name}!</h2>
            <p>Please click the link below to verify your email address:</p>
            <a href="{settings.FRONTEND_URL}/auth/verify/{token}">Verify Email</a>
            <p>If you did not create an account, please ignore this email.</p>
          </body>
        </html>
        """

        # Attach the HTML body to the email
        msg.attach(MIMEText(html, 'html'))

        # Create SMTP session
        server = smtplib.SMTP(settings.MAIL_SERVER, settings.MAIL_PORT)
        server.starttls()  # Enable security
        server.login(settings.MAIL_USERNAME, settings.MAIL_PASSWORD)
        
        # Send email
        text = msg.as_string()
        server.sendmail(settings.MAIL_FROM, email, text)
        server.quit()
        
    except Exception as e:
        # Log the error or handle it appropriately
        print(f"Error sending verification email: {e}")


def send_reset_password_email(email: str, name: str, token: str) -> Any:
    """
    Send password reset email to the user
    """
    try:
        msg = MIMEMultipart()
        msg['From'] = settings.MAIL_FROM
        msg['To'] = email
        msg['Subject'] = "Reset your password"

        # Create the HTML body of the email
        html = f"""
        <html>
          <body>
            <h2>Hello {name},</h2>
            <p>You have requested to reset your password. Please click the link below to reset it:</p>
            <a href="{settings.FRONTEND_URL}/auth/reset-password/{token}">Reset Password</a>
            <p>If you did not request a password reset, please ignore this email.</p>
          </body>
        </html>
        """

        # Attach the HTML body to the email
        msg.attach(MIMEText(html, 'html'))

        # Create SMTP session
        server = smtplib.SMTP(settings.MAIL_SERVER, settings.MAIL_PORT)
        server.starttls()  # Enable security
        server.login(settings.MAIL_USERNAME, settings.MAIL_PASSWORD)
        
        # Send email
        text = msg.as_string()
        server.sendmail(settings.MAIL_FROM, email, text)
        server.quit()
        
    except Exception as e:
        # Log the error or handle it appropriately
        print(f"Error sending reset password email: {e}")