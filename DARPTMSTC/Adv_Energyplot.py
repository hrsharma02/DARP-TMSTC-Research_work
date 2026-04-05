import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

# Load your complete_results.csv
df = pd.read_csv('results/complete_results.csv')

# Pivot to get DARP and baseline energies per configuration
# Create a unique config ID
df['config'] = df['nx'].astype(str) + 'x' + df['ny'].astype(str) + '_d' + df['density'].astype(str) + '_r' + df['robots'].astype(str)

# Get DARP energies
darp = df[df['method'] == 'DARP+TMSTC'][['config', 'energy']].rename(columns={'energy': 'darp_energy'})

# Choose one baseline, e.g., Grid
baseline = df[df['method'] == 'Grid'][['config', 'energy']].rename(columns={'energy': 'grid_energy'})

merged = pd.merge(darp, baseline, on='config')

plt.figure(figsize=(8,8))
plt.scatter(merged['grid_energy'], merged['darp_energy'], alpha=0.7)
plt.plot([0, max(merged.max())], [0, max(merged.max())], 'r--', label='Equal energy')
plt.xlabel('Grid Energy (J)')
plt.ylabel('DARP-TMSTC Energy (J)')
plt.title('DARP-TMSTC vs. Grid: All points below diagonal = DARP better')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig('figures/darp_vs_grid_scatter.pdf')
plt.show()