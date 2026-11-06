# RESONATE project page

GitHub Pages site for [RESONATE](https://github.com/PIC4SeR/RESONATE). It plays the trajectory animation of every benchmark run, selected by scenario, algorithm and run, next to that run's metrics and human ratings.

The videos and `data/runs.json` come from the [RESONATE dataset](https://doi.org/10.5281/zenodo.23763544) (CC BY 4.0). To regenerate them from a local copy of the dataset (needs `ffmpeg`):

```bash
python3 tools/build_site_data.py /path/to/zenodo_resonate
```

To preview locally, serve the folder over HTTP (opening `index.html` as a file blocks loading `runs.json`):

```bash
python3 -m http.server 8000
```
