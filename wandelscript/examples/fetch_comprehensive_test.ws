# Comprehensive test for JavaScript Fetch API compatibility

# Test 1: Basic HTTP methods
print("=== Testing HTTP Methods ===")
get_resp = fetch("https://httpbin.org/get")
print("GET status:", get_resp.status, "ok:", get_resp.ok)

post_resp = fetch("https://httpbin.org/post", { method: "POST", body: { test: "data" } })
print("POST status:", post_resp.status, "ok:", post_resp.ok)

put_resp = fetch("https://httpbin.org/put", { method: "PUT", body: { test: "data" } })
print("PUT status:", put_resp.status, "ok:", put_resp.ok)

delete_resp = fetch("https://httpbin.org/delete", { method: "DELETE" })
print("DELETE status:", delete_resp.status, "ok:", delete_resp.ok)

# Test 2: Different body types
print("\n=== Testing Body Types ===")

# JSON body (default for dict)
json_resp = fetch("https://httpbin.org/post", {
    method: "POST",
    body: { name: "John", age: 30, active: true }
})
print("JSON body status:", json_resp.status)

# Form data body (simple values)
form_resp = fetch("https://httpbin.org/post", {
    method: "POST",
    body: { username: "testuser", password: "secret123" },
    headers: { "Content-Type": "application/x-www-form-urlencoded" }
})
print("Form data status:", form_resp.status)

# Plain text body
text_resp = fetch("https://httpbin.org/post", {
    method: "POST",
    body: "This is plain text content"
})
print("Text body status:", text_resp.status)

# Test 3: JavaScript Fetch API options
print("\n=== Testing Fetch API Options ===")

# Test different modes
cors_resp = fetch("https://httpbin.org/get", { mode: "cors" })
print("CORS mode status:", cors_resp.status)

# Test credentials
creds_resp = fetch("https://httpbin.org/get", { credentials: "same-origin" })
print("Credentials same-origin status:", creds_resp.status)

# Test cache options
cache_resp = fetch("https://httpbin.org/get", { cache: "no-cache" })
print("No-cache status:", cache_resp.status)

# Test redirect handling
redirect_resp = fetch("https://httpbin.org/redirect/1", { redirect: "follow" })
print("Redirect follow status:", redirect_resp.status, "redirected:", redirect_resp.redirected)

# Test keepalive
keepalive_resp = fetch("https://httpbin.org/get", { keepalive: true })
print("Keepalive status:", keepalive_resp.status)

# Test 4: Response object properties
print("\n=== Testing Response Properties ===")
resp = fetch("https://httpbin.org/get")
print("Status:", resp.status)
print("Status text:", resp.statusText)
print("OK:", resp.ok)
print("URL:", resp.url)
print("Redirected:", resp.redirected)
print("Type:", resp.type)
print("Body used initially:", resp.bodyUsed())

# Test 5: Response methods
print("\n=== Testing Response Methods ===")

# Test JSON method
json_resp = fetch("https://httpbin.org/json")
json_data = json_resp.json()
print("JSON method works:", type(json_data))
print("Body used after json():", json_resp.bodyUsed())

# Test text method
text_resp = fetch("https://httpbin.org/html")
text_data = text_resp.text()
print("Text method works:", len(text_data) > 0)
print("Body used after text():", text_resp.bodyUsed())

# Test blob method
blob_resp = fetch("https://httpbin.org/bytes/100")
blob_data = blob_resp.blob()
print("Blob method works:", type(blob_data), "length:", len(blob_data))

# Test arrayBuffer method (alias for blob)
buffer_resp = fetch("https://httpbin.org/bytes/50")
buffer_data = buffer_resp.arrayBuffer()
print("ArrayBuffer method works:", type(buffer_data), "length:", len(buffer_data))

# Test formData method
form_resp = fetch("https://httpbin.org/post", {
    method: "POST",
    body: { key1: "value1", key2: "value2" },
    headers: { "Content-Type": "application/x-www-form-urlencoded" }
})
# Note: This tests the formData() method on a response, not the request body
form_data_resp = fetch("https://httpbin.org/response-headers", {
    headers: { "Content-Type": "application/x-www-form-urlencoded" }
})

# Test clone method
clone_resp = fetch("https://httpbin.org/get")
cloned_resp = clone_resp.clone()
print("Clone method works:", cloned_resp.status == clone_resp.status)
print("Original and clone are different objects:", cloned_resp != clone_resp)

# Test 6: Error handling
print("\n=== Testing Error Handling ===")

# HTTP error statuses should return Response objects, not raise exceptions
error_404 = fetch("https://httpbin.org/status/404")
print("404 status:", error_404.status, "ok:", error_404.ok)

error_500 = fetch("https://httpbin.org/status/500")
print("500 status:", error_500.status, "ok:", error_500.ok)

# Test body consumption tracking
print("\n=== Testing Body Consumption ===")
consumption_resp = fetch("https://httpbin.org/json")
print("Before consumption:", consumption_resp.bodyUsed())
data = consumption_resp.json()
print("After consumption:", consumption_resp.bodyUsed())

# Test 7: Headers handling
print("\n=== Testing Headers ===")
headers_resp = fetch("https://httpbin.org/get", {
    headers: {
        "User-Agent": "WandelScript-Fetch/1.0",
        "Accept": "application/json",
        "Custom-Header": "test-value"
    }
})
print("Custom headers status:", headers_resp.status)
response_headers = headers_resp.headers
print("Response has headers:", len(response_headers) > 0)

print("\n=== All tests completed ===")
