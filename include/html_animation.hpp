#pragma once

#include <fstream>
#include <string>
#include "motion_planner.hpp"
#include "point2d.hpp"
#include "search_space.hpp"

// Writes a self-contained HTML page that replays the tree growing, iteration by
// iteration (like the matplotlib animation in the Python version), including
// RRT* rewires and the best path improving. Open the file in a browser.
inline bool writeAnimation(const std::string& filename, const SearchSpace& space,
                           const TreeHistory& history, const Pose2D& start, const Pose2D& goal,
                           const std::string& title, double scale = 50.0) {
    std::ofstream out(filename);
    if (!out) return false;

    double h = space.getHeight();
    auto sx = [&](const Point2D& p) { return p.getX() * scale; };
    auto sy = [&](const Point2D& p) { return (h - p.getY()) * scale; };  // flip y so +y is up

    out << R"(<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>)" << title << R"( Animation</title>
<style>
  body { font-family: sans-serif; background: #fafafa; color: #111; display: flex; flex-direction: column; align-items: center; margin: 16px; }
  h2 { margin: 4px 0 10px; font-weight: 600; }
  svg { background: white; border: 1px solid #333; max-width: 100%; height: auto; }
  .controls { display: flex; gap: 10px; align-items: center; margin-top: 10px; flex-wrap: wrap; justify-content: center; }
  input[type=range]#scrub { width: min(500px, 80vw); }
  button { padding: 4px 14px; font-size: 15px; }
  #rewired line { stroke: dodgerblue; stroke-width: 2; }
</style></head><body>
<h2 id="title"></h2>
<svg id="canvas" width=")" << space.getWidth() * scale << "\" height=\"" << h * scale
        << "\" viewBox=\"0 0 " << space.getWidth() * scale << " " << h * scale << "\">\n";

    for (const auto& obstacle : space.getObstacles()) {
        out << obstacle->toSvg(scale, h) << "\n";
    }
    out << R"(<g id="tree" stroke="red" stroke-width="1"></g>
<g id="rewired"></g>
<polyline id="path" fill="none" stroke="limegreen" stroke-width="4" points="" />
<circle id="newest" r="4" fill="none" stroke="gray" visibility="hidden" />
)";
    out << "<circle cx=\"" << sx(start.point) << "\" cy=\"" << sy(start.point)
        << "\" r=\"8\" fill=\"blue\"/>\n";
    out << "<circle cx=\"" << sx(goal.point) << "\" cy=\"" << sy(goal.point)
        << "\" r=\"8\" fill=\"gold\" stroke=\"black\"/>\n";
    out << "</svg>\n";
    out << R"(<div class="controls">
  <button id="play">Pause</button>
  <input id="scrub" type="range" min="0" value="0">
  <label>Speed <input id="speed" type="range" min="5" max="600" value="60"></label>
  <span id="speedLabel">60 it/s</span>
</div>
)";

    // Data: node positions (SVG coords) and events as [iteration, node, parent].
    out << "<script>\nconst POINTS = [";
    for (size_t i = 0; i < history.points.size(); ++i) {
        if (i) out << ",";
        out << "[" << sx(history.points[i]) << "," << sy(history.points[i]) << "]";
    }
    out << "];\nconst EVENTS = [";
    for (size_t i = 0; i < history.events.size(); ++i) {
        const auto& e = history.events[i];
        if (i) out << ",";
        out << "[" << e.iteration << "," << e.node << "," << e.parent << "]";
    }
    out << "];\nconst GOAL = " << history.goalIndex << ";\n";
    out << "const TOTAL_ITER = " << history.totalIterations << ";\n";
    out << "const SCALE = " << scale << ";\n";
    out << "const TITLE = \"" << title << "\";\n";

    out << R"(
const SVGNS = "http://www.w3.org/2000/svg";
const treeG = document.getElementById("tree");
const rewiredG = document.getElementById("rewired");
const pathEl = document.getElementById("path");
const newestEl = document.getElementById("newest");
const titleEl = document.getElementById("title");
const scrub = document.getElementById("scrub");
const playBtn = document.getElementById("play");
const speedEl = document.getElementById("speed");
scrub.max = TOTAL_ITER;

let parent = [];   // current parent of each node (undefined = not added yet)
let lines = [];    // SVG line per node, to its parent
let applied = 0;   // number of EVENTS applied
let iter = 0;
let rewires = 0;
let firstCost = null;
let playing = true;
let last = 0;

function setLine(l, a, b) {
  l.setAttribute("x1", POINTS[a][0]); l.setAttribute("y1", POINTS[a][1]);
  l.setAttribute("x2", POINTS[b][0]); l.setAttribute("y2", POINTS[b][1]);
}

function reset() {
  treeG.replaceChildren(); rewiredG.replaceChildren();
  parent = []; lines = []; applied = 0; iter = 0; rewires = 0; firstCost = null;
}

function currentPath() {
  if (GOAL < 0 || parent[GOAL] === undefined) return null;
  const pts = [];
  for (let n = GOAL; n !== -1; n = parent[n]) pts.push(POINTS[n]);
  return pts.reverse();
}

function pathCost(pts) {
  let c = 0;
  for (let i = 1; i < pts.length; i++) c += Math.hypot(pts[i][0] - pts[i-1][0], pts[i][1] - pts[i-1][1]);
  return c / SCALE;
}

function setIteration(target) {
  target = Math.max(0, Math.min(TOTAL_ITER, target));
  if (target < iter) reset();
  iter = target;
  rewiredG.replaceChildren();  // highlight only the latest step's rewires
  let newest = -1;
  while (applied < EVENTS.length && EVENTS[applied][0] <= iter) {
    const [it, n, p] = EVENTS[applied++];
    const isNew = parent[n] === undefined;
    parent[n] = p;
    if (p < 0) continue;
    if (isNew) {
      lines[n] = document.createElementNS(SVGNS, "line");
      treeG.appendChild(lines[n]);
      newest = n;
    } else {
      rewires++;
      if (it === iter) {
        const hl = document.createElementNS(SVGNS, "line");
        setLine(hl, n, p);
        rewiredG.appendChild(hl);
      }
    }
    setLine(lines[n], n, p);
  }
  if (newest >= 0) {
    newestEl.setAttribute("cx", POINTS[newest][0]); newestEl.setAttribute("cy", POINTS[newest][1]);
    newestEl.setAttribute("visibility", "visible");
  }

  const pts = currentPath();
  pathEl.setAttribute("points", pts ? pts.map(p => p.join(",")).join(" ") : "");
  let costText = "";
  if (pts) {
    const c = pathCost(pts);
    if (firstCost === null) firstCost = c;
    costText = `, path cost ${c.toFixed(2)}` + (c < firstCost - 1e-9 ? ` (first ${firstCost.toFixed(2)})` : "");
  }

  const done = iter >= TOTAL_ITER;
  if (done) { newestEl.setAttribute("visibility", "hidden"); rewiredG.replaceChildren(); }
  const nodes = parent.filter(p => p !== undefined).length;
  titleEl.textContent = (done ? `${TITLE} - Complete (${TOTAL_ITER} iterations` : `${TITLE} - Iteration ${iter} (`)
    + `${done ? ", " : ""}${nodes} nodes, ${rewires} rewires${costText}`
    + `${done && !pts ? ", no path found" : ""})`;
  scrub.value = iter;
}

function frame(t) {
  const perSecond = Number(speedEl.value);
  if (playing && t - last >= 1000 / Math.min(perSecond, 60)) {
    const step = Math.max(1, Math.round(perSecond / 60));
    last = t;
    setIteration(iter + step);
    if (iter >= TOTAL_ITER) { playing = false; playBtn.textContent = "Replay"; }
  }
  requestAnimationFrame(frame);
}

playBtn.onclick = () => {
  if (iter >= TOTAL_ITER) { setIteration(0); playing = true; }
  else playing = !playing;
  playBtn.textContent = playing ? "Pause" : "Play";
};
scrub.oninput = () => { playing = false; playBtn.textContent = "Play"; setIteration(Number(scrub.value)); };
speedEl.oninput = () => { document.getElementById("speedLabel").textContent = speedEl.value + " it/s"; };

setIteration(0);
requestAnimationFrame(frame);
</script>
</body></html>
)";
    return true;
}
