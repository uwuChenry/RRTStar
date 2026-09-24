#pragma once

#include <fstream>
#include <string>
#include <vector>
#include "point2d.hpp"
#include "rrt.hpp"
#include "search_space.hpp"

// Writes a self-contained HTML page that replays the RRT tree growing,
// iteration by iteration (like the matplotlib animation in the Python version).
// Open the file in a browser: play/pause, change speed, or drag the slider.
inline bool writeAnimation(const std::string& filename, const SearchSpace& space,
                           const std::vector<Node>& tree, const std::vector<Point2D>& path,
                           const Pose2D& start, const Pose2D& goal, int totalIterations,
                           const std::string& title = "RRT", double scale = 50.0) {
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
</style></head><body>
<h2 id="title"></h2>
<svg id="canvas" width=")" << space.getWidth() * scale << "\" height=\"" << h * scale
        << "\" viewBox=\"0 0 " << space.getWidth() * scale << " " << h * scale << "\">\n";

    for (const auto& obstacle : space.getObstacles()) {
        out << obstacle->toSvg(scale, h) << "\n";
    }
    out << R"(<g id="tree" stroke="red" stroke-width="1"></g>
<polyline id="path" fill="none" stroke="limegreen" stroke-width="4" points="" />
<circle id="sample" r="4" fill="none" stroke="gray" visibility="hidden" />
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

    // Data: nodes as [x, y, parentIndex, iteration], in insertion order.
    out << "<script>\nconst NODES = [";
    for (size_t i = 0; i < tree.size(); ++i) {
        if (i) out << ",";
        out << "[" << sx(tree[i].point) << "," << sy(tree[i].point) << ","
            << tree[i].parent << "," << tree[i].iteration << "]";
    }
    out << "];\nconst PATH = [";
    for (size_t i = 0; i < path.size(); ++i) {
        if (i) out << ",";
        out << "[" << sx(path[i]) << "," << sy(path[i]) << "]";
    }
    out << "];\nconst TOTAL_ITER = " << totalIterations << ";\n";
    out << "const TITLE = \"" << title << "\";\n";

    out << R"(
const SVGNS = "http://www.w3.org/2000/svg";
const treeG = document.getElementById("tree");
const pathEl = document.getElementById("path");
const sampleEl = document.getElementById("sample");
const titleEl = document.getElementById("title");
const scrub = document.getElementById("scrub");
const playBtn = document.getElementById("play");
const speedEl = document.getElementById("speed");
scrub.max = TOTAL_ITER;

let shown = 1;        // number of nodes currently drawn (node 0 = start)
let iter = 0;
let playing = true;
let last = 0;

function addEdge(i) {
  const [x, y, p] = NODES[i];
  if (p < 0) return;
  const l = document.createElementNS(SVGNS, "line");
  l.setAttribute("x1", x); l.setAttribute("y1", y);
  l.setAttribute("x2", NODES[p][0]); l.setAttribute("y2", NODES[p][1]);
  treeG.appendChild(l);
}

function setIteration(target) {
  target = Math.max(0, Math.min(TOTAL_ITER, target));
  if (target < iter) { treeG.replaceChildren(); shown = 1; }
  iter = target;
  let newest = -1;
  while (shown < NODES.length && NODES[shown][3] <= iter) { addEdge(shown); newest = shown; shown++; }
  if (newest >= 0) {
    sampleEl.setAttribute("cx", NODES[newest][0]); sampleEl.setAttribute("cy", NODES[newest][1]);
    sampleEl.setAttribute("visibility", "visible");
  }
  const done = iter >= TOTAL_ITER;
  pathEl.setAttribute("points", done ? PATH.map(p => p.join(",")).join(" ") : "");
  if (done) sampleEl.setAttribute("visibility", "hidden");
  titleEl.textContent = done
    ? `${TITLE} - Complete (${TOTAL_ITER} iterations, ${NODES.length} nodes${PATH.length ? "" : ", no path found"})`
    : `${TITLE} - Iteration ${iter}`;
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
)";
    out << "</body></html>\n";
    return true;
}
