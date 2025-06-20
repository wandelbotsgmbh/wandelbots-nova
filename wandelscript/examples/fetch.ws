res_get = fetch("https://httpbin.org/get")
print(res_get)

res_get_error = fetch("https://httpbin.org/unknown")
print(res_get_error)

res_post = fetch("https://httpbin.org/post", {
    method: "POST",
    body: { name: "JohnDoe" },
})
print(res_post)
