const BACKEND_URL = "https://fuzzy-space-spoon-q77rggjpg5rxc4wq4-8080.app.github.dev";

export async function runSimulation(action) {
  const response = await fetch(`${BACKEND_URL}/control`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ action }),
  });

  if (!response.ok) {
    throw new Error(`Server error: ${response.status}`);
  }
  return await response.json();
}


export async function getSimulationState() {
  try {
    const response = await fetch('/state', {
      method: 'GET',
      headers: { 'Content-Type': 'application/json' },
    });

    if (!response.ok) {
      throw new Error(`Failed to fetch state: ${response.status}`);
    }

    return await response.json();
  } catch (error) {
    console.error('Error fetching simulation state:', error);
    throw error;
  }
}
