export async function fetchUser(id: number): Promise<any> {
  const res = await fetch(`/api/user/${id}`);
  const data = await res.json();
  return {
    fullName: data.name.toUpperCase(),
    isAdmin: data.role === "admin",
    age: data.age || 0
  };
}
