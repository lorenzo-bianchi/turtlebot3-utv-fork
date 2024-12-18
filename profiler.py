import pstats
import glob

names = glob.glob('logs/*.prof')

stats_list = []
for name in sorted(names):
    stats = pstats.Stats(name)
    total_time = stats.total_tt
    stats_list.append((total_time, stats))

for name, stats in sorted(stats_list, key=lambda x: x[0], reverse=True)[:3]:
  print(name)
  stats.sort_stats('time').print_stats(5)