import asyncio

from nova.auth.authorization import Auth0DeviceAuthorization

"""
Example: Perform device authorization with Auth0.

"""


async def main():
    auth0_device_auth = Auth0DeviceAuthorization()

    # or login using the NOVA default client_id, domain and audience
    # auth0_device_auth = Auth0DeviceAuthorization()

    try:
        # Request a device code
        device_code_info = auth0_device_auth.request_device_code()
        print("Device code requested successfully.")
        print(f"User Code: {device_code_info.user_code}")
        print(f"Verification URI: {device_code_info.verification_uri}")

        # Display user instructions
        auth0_device_auth.display_user_instructions()

        # Poll the token endpoint to obtain an access token
        access_token = await auth0_device_auth.poll_token_endpoint()
        print(f"Access Token: {access_token}")

    except Exception as e:
        print(f"An error occurred: {e}")


if __name__ == "__main__":
    asyncio.run(main())
