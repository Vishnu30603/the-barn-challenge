import numpy as np
import matplotlib
matplotlib.use('Agg')  # no display needed
import matplotlib.pyplot as plt

def load_results(filename):
    results = {}
    try:
        with open(filename) as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) == 6:
                    world_idx = int(parts[0])
                    success = int(parts[1])
                    collided = int(parts[2])
                    timeout = int(parts[3])
                    actual_time = float(parts[4])
                    nav_metric = float(parts[5])
                    results[world_idx] = {
                        'success': success,
                        'collided': collided,
                        'timeout': timeout,
                        'actual_time': actual_time,
                        'nav_metric': nav_metric
                    }
    except:
        print(f"Could not load {filename}")
    return results

dwa = load_results('dwa_results.txt')
teb = load_results('teb_results.txt')
ada = load_results('adaptive_results.txt')

worlds = sorted(set(list(dwa.keys()) + list(teb.keys()) + list(ada.keys())))

# Extract metrics
def get_metrics(results, worlds):
    scores = [results.get(w, {}).get('nav_metric', 0) for w in worlds]
    successes = [results.get(w, {}).get('success', 0) for w in worlds]
    return scores, successes

dwa_scores, dwa_succ = get_metrics(dwa, worlds)
teb_scores, teb_succ = get_metrics(teb, worlds)
ada_scores, ada_succ = get_metrics(ada, worlds)

# Plot 1 - Per world scores
plt.figure(figsize=(14, 5))
x = np.arange(len(worlds))
w = 0.25
plt.bar(x - w, dwa_scores, w, label='DWA', color='#E24B4A')
plt.bar(x, teb_scores, w, label='TEB', color='#378ADD')
plt.bar(x + w, ada_scores, w, label='Adaptive TEB', color='#1D9E75')
plt.xlabel('World Index')
plt.ylabel('Navigation Score')
plt.title('Navigation Score per Environment')
plt.xticks(x, worlds)
plt.legend()
plt.tight_layout()
plt.savefig('plot_per_world.png', dpi=150)
print("Saved plot_per_world.png")

# Plot 2 - Average scores
plt.figure(figsize=(6, 5))
avgs = [np.mean(dwa_scores), np.mean(teb_scores), np.mean(ada_scores)]
colors = ['#E24B4A', '#378ADD', '#1D9E75']
bars = plt.bar(['DWA', 'TEB', 'Adaptive TEB'], avgs, color=colors)
plt.ylabel('Average Navigation Score')
plt.title('Average Score Comparison')
for bar, val in zip(bars, avgs):
    plt.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.005,
             f'{val:.3f}', ha='center', fontsize=11)
plt.tight_layout()
plt.savefig('plot_avg_score.png', dpi=150)
print("Saved plot_avg_score.png")

# Plot 3 - Success rate
plt.figure(figsize=(6, 5))
rates = [sum(dwa_succ)/len(worlds)*100,
         sum(teb_succ)/len(worlds)*100,
         sum(ada_succ)/len(worlds)*100]
bars = plt.bar(['DWA', 'TEB', 'Adaptive TEB'], rates, color=colors)
plt.ylabel('Success Rate (%)')
plt.title('Success Rate Comparison')
plt.ylim(0, 100)
for bar, val in zip(bars, rates):
    plt.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1,
             f'{val:.1f}%', ha='center', fontsize=11)
plt.tight_layout()
plt.savefig('plot_success_rate.png', dpi=150)
print("Saved plot_success_rate.png")

# Print summary
print("\n========== RESULTS SUMMARY ==========")
print(f"{'Approach':<20} {'Avg Score':<15} {'Success Rate':<15}")
print("-" * 50)
print(f"{'DWA':<20} {np.mean(dwa_scores):<15.4f} {sum(dwa_succ)}/{len(worlds)}")
print(f"{'TEB':<20} {np.mean(teb_scores):<15.4f} {sum(teb_succ)}/{len(worlds)}")
print(f"{'Adaptive TEB':<20} {np.mean(ada_scores):<15.4f} {sum(ada_succ)}/{len(worlds)}")
