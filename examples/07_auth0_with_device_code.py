from nova.auth.authorization import Auth0DeviceAuthorization

"""
Example: Perform device authorization with Auth0.

Prerequisites:
- Replace 'YOUR-AUTH0-DOMAIN', 'YOUR-AUTH0-CLIENT-ID', and 'YOUR-AUTH0-AUDIENCE' with actual values.
"""


def main():
    # Replace these values with your Auth0 domain, client ID, and audience
    auth0_domain = "YOUR-AUTH0-DOMAIN"
    auth0_client_id = "YOUR-AUTH0-CLIENT-ID"
    auth0_audience = "YOUR-AUTH0-AUDIENCE"

    # Initialize the Auth0DeviceAuthorization instance
    auth0_device_auth = Auth0DeviceAuthorization(auth0_domain, auth0_client_id, auth0_audience)

    try:
        # Request a device code
        device_code_info = auth0_device_auth.request_device_code()
        print("Device code requested successfully.")
        print(f"User Code: {device_code_info.user_code}")
        print(f"Verification URI: {device_code_info.verification_uri}")

        # Display user instructions
        auth0_device_auth.display_user_instructions()

        # Poll the token endpoint to obtain an access token
        access_token = auth0_device_auth.poll_token_endpoint()
        print(f"Access Token: {access_token}")

        # Optionally, refresh the access token using the refresh token
        if auth0_device_auth.refresh_token:
            new_access_token = auth0_device_auth.refresh_access_token(
                auth0_device_auth.refresh_token
            )
            print(f"New Access Token: {new_access_token}")

    except Exception as e:
        print(f"An error occurred: {e}")


if __name__ == "__main__":
    main()
