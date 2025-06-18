# Test basic GET request with new Response object format
res_get = fetch("https://httpbin.org/get")
print("GET Response status:", res_get.status)
print("GET Response ok:", res_get.ok)
print("GET Response headers type:", type(res_get.headers))
print("GET Response data:", res_get.json())

# Test 404 error - should return Response object with ok=false, not raise exception
res_get_error = fetch("https://httpbin.org/status/404")
print("404 Response status:", res_get_error.status)
print("404 Response ok:", res_get_error.ok)
print("404 Response statusText:", res_get_error.statusText)

# Test POST request with JSON body
res_post = fetch("https://httpbin.org/post", {
    method: "POST",
    body: { name: "JohnDoe" },
})
print("POST Response status:", res_post.status)
print("POST Response ok:", res_post.ok)
post_data = res_post.json()
print("POST Response json data:", post_data.json)

# Test different body types
# Test form data (URLSearchParams-like)
res_form = fetch("https://httpbin.org/post", {
    method: "POST",
    body: { username: "test", password: "secret" },
    headers: { "Content-Type": "application/x-www-form-urlencoded" }
})
print("Form POST status:", res_form.status)

# Test plain text body
res_text = fetch("https://httpbin.org/post", {
    method: "POST",
    body: "Hello, World!",
})
print("Text POST status:", res_text.status)

# Test new Fetch API options
res_options = fetch("https://httpbin.org/get", {
    method: "GET",
    mode: "cors",
    credentials: "same-origin",
    cache: "default",
    redirect: "follow"
})
print("Options GET status:", res_options.status)
print("Options GET redirected:", res_options.redirected)

# Test response methods
res_methods = fetch("https://httpbin.org/json")
print("JSON endpoint status:", res_methods.status)
json_data = res_methods.json()
print("JSON data type:", type(json_data))

# Test text response
res_text_resp = fetch("https://httpbin.org/html")
print("HTML endpoint status:", res_text_resp.status)
html_content = res_text_resp.text()
print("HTML content length:", len(html_content))

# Test clone method
res_clone_test = fetch("https://httpbin.org/get")
res_cloned = res_clone_test.clone()
print("Original status:", res_clone_test.status)
print("Cloned status:", res_cloned.status)

# Test bodyUsed tracking
res_body_used = fetch("https://httpbin.org/json")
print("Body used before:", res_body_used.bodyUsed())
data = res_body_used.json()
print("Body used after:", res_body_used.bodyUsed())
