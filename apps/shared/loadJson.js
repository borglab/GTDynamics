// Accept both gzip archives and responses already decoded by the HTTP server.
export async function loadJson(path, signal, compressed = false) {
  const response = await fetch(`${import.meta.env.BASE_URL}${path}`, {
    signal,
  });
  if (!response.ok)
    throw new Error(`Could not load ${path} (${response.status})`);
  if (!compressed) return response.json();
  let buffer = await response.arrayBuffer();
  const bytes = new Uint8Array(buffer);
  if (bytes[0] === 31 && bytes[1] === 139)
    buffer = await new Response(
      new Blob([buffer]).stream().pipeThrough(new DecompressionStream("gzip")),
    ).arrayBuffer();
  return JSON.parse(new TextDecoder().decode(buffer));
}
