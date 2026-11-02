import assert from "node:assert/strict";
import { existsSync, readFileSync } from "node:fs";
import { gunzipSync } from "node:zlib";
import { fileURLToPath } from "node:url";
import { preview } from "vite";
const { chromium } = await import(
  process.env.PLAYWRIGHT_MODULE || "playwright"
);
const record = JSON.parse(
  gunzipSync(
    readFileSync(new URL("../public/snippets/onset.json.gz", import.meta.url)),
  ),
);
const inferredSide = ["left", "right"].find((side) =>
  record.feet[side].targets.default.walkingLabels.includes(2),
);
const inferredFrame =
  record.feet[inferredSide].targets.default.walkingLabels.indexOf(2);
const root = fileURLToPath(new URL("../", import.meta.url));
const browser = await chromium.launch({
  executablePath:
    process.env.CHROME_EXECUTABLE ||
    (existsSync("/usr/bin/google-chrome")
      ? "/usr/bin/google-chrome"
      : undefined),
  headless: true,
  args: ["--no-sandbox", "--enable-unsafe-swiftshader"],
});
try {
  for (const base of ["/", "/GTDynamics/contact-labels/"]) {
    const server = await preview({
      root,
      configFile: false,
      base,
      preview: { host: "127.0.0.1", port: 0, strictPort: true },
    });
    const page = await browser.newPage({
      viewport: { width: 1460, height: 1050 },
    });
    page.setDefaultTimeout(60000);
    const errors = [],
      failures = [],
      requests = [];
    page.on("pageerror", (e) => errors.push(e.message));
    page.on("request", (r) => requests.push(r.url()));
    page.on("response", (r) => {
      if (r.status() >= 400 && !r.url().endsWith("favicon.ico"))
        failures.push(r.url());
    });
    const ready = () =>
      page.locator('.label-scene[data-ready="true"]').waitFor();
    try {
      await page.goto(
        `http://127.0.0.1:${server.httpServer.address().port}${base}`,
      );
      await ready();
      assert.equal(
        await page
          .getByRole("heading", { name: "Contact Labels", exact: true })
          .count(),
        1,
      );
      assert.equal(await page.locator("canvas").count(), 1);
      assert.equal(
        await page
          .getByRole("button", { name: "Follow robot", exact: true })
          .getAttribute("aria-pressed"),
        "true",
      );
      if (base === "/")
        await page.screenshot({
          path: "/tmp/gtd-contact-labels-follow.png",
          fullPage: false,
        });
      assert.equal(await page.locator(".foot-card").count(), 2);
      assert.equal(
        requests.filter((url) => /snippets\/.*\.json\.gz$/.test(url)).length,
        1,
      );
      await page.waitForFunction(
        () =>
          document.querySelectorAll('.chart[data-ready="true"]').length === 7,
      );
      const time = page.getByRole("slider", { name: "Clip time", exact: true });
      const bandStyles = await page
        .locator(".label-band:not(.comparison-band) button")
        .evaluateAll((buttons) =>
          buttons.map((button) => {
            const style = getComputedStyle(button);
            return {
              color: style.backgroundColor,
              shadow: style.boxShadow,
              opacity: style.opacity,
            };
          }),
        );
      assert(bandStyles.length > 0);
      assert(
        bandStyles.every(
          (style) =>
            ["rgb(0, 0, 0)", "rgb(255, 255, 255)"].includes(style.color) &&
            style.shadow === "none" &&
            style.opacity === "1",
        ),
      );
      const model = page.getByRole("combobox", {
        name: "Label model",
        exact: true,
      });
      assert.equal(await model.inputValue(), "graph");
      assert.equal(
        await page.locator(".label-scene").getAttribute("data-method"),
        "graph",
      );
      assert.equal(await page.locator(".comparison-band").count(), 3);
      await time.fill("400");
      const support = record.supportGraph.states[400];
      for (const side of ["left", "right"]) {
        const expected =
          support === (side === "left" ? 1 : 0) ? "Swing (FG)" : "Support (FG)";
        assert(
          (
            await page.locator(`[data-foot="${side}"] .row`).innerText()
          ).includes(expected),
        );
      }
      const aiding = page.getByRole("checkbox", {
        name: "Joint-velocity factors",
        exact: true,
      });
      assert(await aiding.isChecked());
      await aiding.uncheck();
      assert.equal(
        await page.locator(".label-scene").getAttribute("data-velocity-aiding"),
        "false",
      );
      await aiding.check();
      assert.equal(
        await page.locator(".label-scene").getAttribute("data-velocity-aiding"),
        "true",
      );
      const constraints = page.getByRole("checkbox", {
        name: "Require support and double-stance transitions", exact: true,
      });
      assert(await constraints.isChecked());
      const flightFrame = record.supportGraph.withoutWalkingConstraints.states.indexOf(3);
      assert(flightFrame >= 0, "unconstrained example must exercise both-swing");
      await time.fill(String(flightFrame));
      await constraints.uncheck();
      assert.equal(await page.locator(".label-scene").getAttribute("data-walking-constraints"), "false");
      for (const side of ["left", "right"])
        assert((await page.locator(`[data-foot="${side}"] .row`).innerText()).includes("Swing (FG)"));
      for (const velocity of [false, true]) {
        await aiding.setChecked(velocity);
        await constraints.check();
        assert.equal(await page.locator(".label-scene").getAttribute("data-walking-constraints"), "true");
        const chosen = velocity ? record.supportGraph : record.supportGraph.withoutVelocity;
        const state = chosen.states[flightFrame];
        for (const side of ["left", "right"]) {
          const expected = state === (side === "left" ? 1 : 0) ? "Swing (FG)" : "Support (FG)";
          assert((await page.locator(`[data-foot="${side}"] .row`).innerText()).includes(expected));
        }
        await constraints.uncheck();
      }
      await constraints.check();
      assert((await page.locator('.contact-mathematics mjx-container[jax="SVG"]').count()) >= 10);
      assert.equal(await page.locator('[data-mml-node="merror"]').count(), 0);
      assert.equal(await page.locator('a[href*="contact-factors.html"]').count(), 0);
      assert.equal(await page.locator('[data-math-parameters="contact"] table').count(), 4);
      const parameters = await page.locator('[data-math-parameters="contact"]').innerText();
      assert(parameters.includes(`${record.supportGraph.supportSigma} m`));
      assert(parameters.includes(`${record.supportGraph.factorCount} factors`));
      assert.match(await page.locator('.contact-mathematics').innerText(), /density values evaluated at the same observed displacement/);
      assert(!requests.some(url => /mathjax|cdn\./i.test(new URL(url).hostname)));
      assert.equal(await page.locator(".graph-comparison details").count(), 0);
      if (base === "/")
        await page
          .locator(".graph-comparison")
          .screenshot({ path: "/tmp/gtd-contact-labels-graph.png" });
      await model.selectOption("window");
      assert.equal(
        await page.locator(".label-scene").getAttribute("data-method"),
        "window",
      );
      const prior = page.getByRole("checkbox", {
        name: "Walking support prior",
        exact: true,
      });
      assert(await prior.isChecked());
      await time.fill(String(inferredFrame));
      assert.match(
        await page.locator(`[data-foot="${inferredSide}"] .row`).innerText(),
        /Likely contact/,
      );
      const rawScore = await page
        .locator(".chart")
        .first()
        .evaluate((div) => div.data[0].y.slice());
      await prior.uncheck();
      assert.doesNotMatch(
        await page.locator(`[data-foot="${inferredSide}"] .row`).innerText(),
        /Likely contact/,
      );
      assert.equal(
        await page.locator(".label-scene").getAttribute("data-walking-prior"),
        "false",
      );
      assert.deepEqual(
        await page
          .locator(".chart")
          .first()
          .evaluate((div) => div.data[0].y.slice()),
        rawScore,
      );
      await prior.check();
      assert.equal(
        await page.locator(".label-scene").getAttribute("data-walking-prior"),
        "true",
      );
      if (base === "/")
        await page.screenshot({
          path: "/tmp/gtd-contact-labels-support.png",
          fullPage: false,
        });
      await prior.uncheck();
      await time.fill("225");
      assert.equal(
        await page.locator("main").getAttribute("data-frame"),
        "225",
      );
      assert.match(
        await page.locator('[data-foot="right"] .row').innerText(),
        /Moving/,
      );
      await prior.check();
      const threshold = page.getByRole("combobox", {
        name: "Motion thresholds",
        exact: true,
      });
      await threshold.selectOption("strict");
      assert.equal(
        await page.locator(".label-scene").getAttribute("data-preset"),
        "strict",
      );
      await threshold.selectOption("tolerant");
      assert.equal(
        await page.locator("main").getAttribute("data-frame"),
        "225",
      );
      const trails = page.getByRole("checkbox", {
        name: "Contact-frame trails",
        exact: true,
      });
      await trails.uncheck();
      await trails.check();
      const chart = page.locator(".chart").first();
      await chart.scrollIntoViewIfNeeded();
      const point = await chart.evaluate((div) => ({
        x:
          div._fullLayout.xaxis._offset +
          div._fullLayout.xaxis.l2p(div.data[0].x[500]),
        y:
          div._fullLayout.yaxis._offset +
          div._fullLayout.yaxis.l2p(div.data[0].y[500]),
      }));
      await chart.click({ position: point });
      await page.waitForFunction(
        () =>
          Math.abs(Number(document.querySelector("main").dataset.frame) - 500) <
          5,
      );
      const drag = await chart.locator(".nsewdrag").boundingBox();
      await page.mouse.move(
        drag.x + drag.width * 0.2,
        drag.y + drag.height * 0.2,
      );
      await page.mouse.down();
      await page.mouse.move(
        drag.x + drag.width * 0.6,
        drag.y + drag.height * 0.8,
        { steps: 8 },
      );
      await page.mouse.up();
      await page.waitForFunction(() => {
        const charts = [...document.querySelectorAll(".chart")];
        const a = charts[0]._fullLayout.xaxis.range;
        return (
          a[1] - a[0] < 15 &&
          charts.every((c) =>
            c._fullLayout.xaxis.range.every(
              (v, i) => Math.abs(v - a[i]) < 1e-6,
            ),
          )
        );
      });
      await page
        .getByRole("button", { name: "Reset plot zoom", exact: true })
        .click();
      await page.waitForFunction(
        () =>
          Math.abs(
            document.querySelector(".chart")._fullLayout.xaxis.range[0] - 24,
          ) < 1e-8,
      );
      await model.selectOption("graph");
      assert.equal(
        await page.locator(".label-scene").getAttribute("data-method"),
        "graph",
      );
      const snippet = page.getByRole("combobox", {
        name: "Snippet",
        exact: true,
      });
      for (const value of ["middle", "return", "onset"]) {
        await snippet.selectOption(value);
        await ready();
        assert.equal(
          await page.locator("main").getAttribute("data-snippet"),
          value,
        );
        assert.equal(await page.locator("canvas").count(), 1);
      }
      await page
        .getByRole("button", { name: "Follow robot", exact: true })
        .click();
      await page.getByRole("button", { name: "Play", exact: true }).click();
      await page.waitForTimeout(700);
      await page.getByRole("button", { name: "Pause", exact: true }).click();
      assert(
        Number(await page.locator("main").getAttribute("data-frame")) > 20,
      );
      await page
        .getByRole("button", { name: "Fit snippet", exact: true })
        .click();
      await page.waitForFunction(
        () =>
          document.querySelectorAll('.chart[data-ready="true"]').length === 7,
      );
      if (base === "/")
        await page.screenshot({
          path: "/tmp/gtd-contact-labels.png",
          fullPage: true,
        });
      await page.setViewportSize({ width: 390, height: 844 });
      await page.waitForTimeout(350);
      assert(
        await page.evaluate(
          () => document.documentElement.scrollWidth <= innerWidth + 1,
        ),
      );
      if (base === "/")
        await page.screenshot({
          path: "/tmp/gtd-contact-labels-mobile.png",
          fullPage: true,
        });
      assert.deepEqual(errors, []);
      assert.deepEqual(failures, []);
      console.log(`Passed contact labels ${base}`);
    } finally {
      await page.close();
      server.httpServer.closeAllConnections();
      await new Promise((resolve) => server.httpServer.close(resolve));
    }
  }
  const hub = await preview({
    root: fileURLToPath(new URL("../../", import.meta.url)),
    preview: { host: "127.0.0.1", port: 0, strictPort: true },
  });
  const page = await browser.newPage();
  try {
    await page.goto(
      `http://127.0.0.1:${hub.httpServer.address().port}/#contact-labels`,
    );
    await page
      .frameLocator("iframe")
      .locator('.label-scene[data-ready="true"]')
      .waitFor({ timeout: 60000 });
    await page.getByRole("tab", { name: "Overview", exact: true }).click();
    await page.locator("iframe").waitFor({ state: "detached" });
    await page
      .getByRole("tab", { name: "Contact Labels", exact: true })
      .click();
    await page
      .frameLocator("iframe")
      .locator('.label-scene[data-ready="true"]')
      .waitFor({ timeout: 60000 });
    console.log("Passed hub deep link, unload, and reopen");
  } finally {
    await page.close();
    hub.httpServer.closeAllConnections();
    await new Promise((resolve) => hub.httpServer.close(resolve));
  }
} finally {
  await browser.close();
}
