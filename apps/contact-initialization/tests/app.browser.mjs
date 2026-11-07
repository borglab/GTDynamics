import assert from "node:assert/strict";
import { existsSync } from "node:fs";
import { readFile } from "node:fs/promises";
import { fileURLToPath } from "node:url";
import { preview } from "vite";
const { chromium } = await import(
  process.env.PLAYWRIGHT_MODULE || "playwright"
);
const root = fileURLToPath(new URL("../../", import.meta.url));
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
  for (const base of ["/", "/GTDynamics/"]) {
    const server = await preview({
      root,
      configFile: false,
      base,
      preview: { host: "127.0.0.1", port: 0, strictPort: true },
    });
    const page = await browser.newPage({
      viewport: { width: 1500, height: 1100 },
      acceptDownloads: true,
    });
    page.setDefaultTimeout(60000);
    const errors = [],
      failures = [];
    page.on("pageerror", (e) => errors.push(e.message));
    page.on("response", (r) => {
      if (r.status() >= 400 && !r.url().endsWith("favicon.ico"))
        failures.push(r.url());
    });
    try {
      await page.goto(
        `http://127.0.0.1:${server.httpServer.address().port}${base}#contact-initialization`,
      );
      const app = page.frameLocator("iframe");
      await app.locator('.scene[data-renderer="mujoco"]').waitFor();
      const lab = app.locator(".lab");
      assert.equal(await lab.getAttribute("data-model"), "pose");
      assert.equal(
        await app.getByRole("combobox", { name: "Contact geometry" }).count(),
        0,
      );
      assert.equal(
        await app.locator(".configuration-table tbody tr").count(),
        8,
      );
      assert.equal(
        await lab.getAttribute("data-variant"),
        "between-ground-upright",
      );
      assert.equal(await lab.getAttribute("data-sequence"), "ssio-onset");
      assert.match(
        await app.locator(".headline-metrics").innerText(),
        /stalled: no accepted decrease/,
      );
      const viewerBounds = await app.locator(".scene").boundingBox();
      const graphBounds = await app.locator(".pose-graph").boundingBox();
      const playbackBounds = await app.locator(".playback").boundingBox();
      const controlsBounds = await app.locator(".prior-controls").boundingBox();
      assert(graphBounds.y > viewerBounds.y + viewerBounds.height);
      assert.equal(await app.locator('[data-paper-section="approach"] .pose-graph').count(), 1);
      assert(controlsBounds.y >= playbackBounds.y + playbackBounds.height);
      assert((await app.locator("svg .upright-factor").count()) > 0);
      await app
        .getByRole("checkbox", { name: "Pelvis upright prior" })
        .uncheck();
      assert.equal(await app.locator("svg .upright-factor").count(), 0);
      assert.equal(
        await app.locator(".scene").getAttribute("data-joints"),
        "29",
      );
      assert.match(
        await app.locator(".graph-stats").innerText(),
        /contact poses/,
      );
      for (const [id, motion, ground] of [
        ["none", false, false],
        ["between", true, false],
        ["between-ground", true, true],
        ["ground", false, true],
      ]) {
        await app
          .getByRole("checkbox", { name: "Weak motion prior" })
          .setChecked(motion);
        await app
          .getByRole("checkbox", { name: "Sole ground priors" })
          .setChecked(ground);
        await app.locator(`.lab[data-variant="${id}"]`).waitFor();
        assert.equal(
          (await app.locator("svg .motion-factor").count()) > 0,
          motion,
        );
        assert.equal(
          (await app.locator("svg .height-factor").count()) > 0,
          ground,
        );
      }
      await app.getByRole("checkbox", { name: "Pelvis upright prior" }).check();
      await app.locator('.lab[data-variant="ground-upright"]').waitFor();
      assert((await app.locator("svg .upright-factor").count()) > 0);
      if (base === "/")
        await page.screenshot({
          path: "/tmp/contact-graph-layout.png",
          fullPage: false,
        });
      const slider = app.getByRole("slider", {
        name: "Contact event",
        exact: true,
      });
      const graph = app.locator(".pose-graph svg");
      const event = (i) => graph.getByRole("button", { name: `Select event ${i}`, exact: true });
      await event(1).click();
      assert.equal(await slider.inputValue(), "1");
      await event(2).press("Enter");
      assert.equal(await slider.inputValue(), "2");
      await event(3).press("Space");
      assert.equal(await slider.inputValue(), "3");
      assert.equal(await graph.locator(".contact-node[tabindex]").count(), 0);

      await slider.fill("20");
      assert.equal(await lab.getAttribute("data-event"), "20");
      assert.equal(await event(20).getAttribute("aria-pressed"), "true");
      const active = graph.locator('[data-factor][data-active="true"]');
      assert(await active.count() > 0);
      assert.equal(await active.count(), await active.and(graph.locator(".observation-factor")).count());
      await app
        .getByRole("button", { name: "Play event playback", exact: true })
        .click();
      await page.waitForFunction(
        () =>
          Number(
            document
              .querySelector("iframe")
              .contentDocument.querySelector(".lab").dataset.event,
          ) !== 20,
      );
      await app
        .getByRole("button", { name: "Pause event playback", exact: true })
        .evaluate((b) => b.click());

      await app
        .getByRole("combobox", { name: "Display trajectory" })
        .selectOption("reference");
      await app
        .getByRole("combobox", { name: "Display trajectory" })
        .selectOption("estimate");
      await app
        .getByRole("checkbox", { name: "Reference", exact: true })
        .uncheck();
      await app
        .getByRole("checkbox", { name: "Reference", exact: true })
        .check();
      await app.getByRole("button", { name: "Top", exact: true }).click();
      await app
        .getByRole("button", { name: "Fit trajectory", exact: true })
        .click();
      const iteration = app.getByRole("slider", { name: "Solver iteration" });
      await iteration.fill("0");
      const initialBudget = await app.locator(".factor-table").innerText();
      await iteration.fill(await iteration.getAttribute("max"));
      assert.notEqual(
        await app.locator(".factor-table").innerText(),
        initialBudget,
      );
      const download = page.waitForEvent("download");
      await app
        .getByRole("button", { name: "Export initialization JSON" })
        .click();
      const file = await download,
        payload = JSON.parse(await readFile(await file.path(), "utf8"));
      assert.equal(
        payload.solver,
        "Initializer.InitializeSolutionContactPoses",
      );
      assert.equal(
        payload.result.contactPoses.length,
        payload.graph.contacts.length,
      );
      assert.equal(payload.result.landmarks, undefined);
      for (const id of ["ssio-middle", "ssio-return"]) {
        await app
          .getByRole("combobox", { name: "Motion sequence" })
          .selectOption(id);
        await app
          .locator('select[aria-label="Motion sequence"]:not(:disabled)')
          .waitFor();
        await app
          .locator(
            `.lab[data-sequence="${id}"][data-variant="between-ground-upright"]`,
          )
          .waitFor();
        await app
          .getByRole("checkbox", { name: "Pelvis upright prior" })
          .waitFor();
        await app
          .getByRole("button", { name: "Fit trajectory", exact: true })
          .click();
        if (base === "/" && id === "ssio-middle")
          await page.screenshot({
            path: "/tmp/contact-middle-fit.png",
            fullPage: false,
          });
        assert.equal(
          await app.locator(".configuration-table tbody tr").count(),
          8,
        );
      }
      if (base === "/")
        await page.screenshot({
          path: "/tmp/ssio-contact-initialization.png",
          fullPage: true,
        });
      await app
        .getByRole("combobox", { name: "Motion sequence" })
        .selectOption("g1-policy-8s");
      await app
        .locator(
          '.lab[data-sequence="g1-policy-8s"][data-variant="between-ground-upright"]',
        )
        .waitFor();
      await app
        .locator('select[aria-label="Motion sequence"]:not(:disabled)')
        .waitFor();
      assert.equal(
        await app.locator(".configuration-table tbody tr").count(),
        8,
      );
      await app.getByRole("checkbox", { name: "Weak motion prior" }).uncheck();
      await app.getByRole("checkbox", { name: "Sole ground priors" }).uncheck();
      await app
        .getByRole("checkbox", { name: "Pelvis upright prior" })
        .uncheck();
      await app.locator('.lab[data-variant="none"]').waitFor();
      await app
        .getByRole("combobox", { name: "Motion sequence" })
        .selectOption("g1-steptouch-30s");
      await app
        .locator(
          '.lab[data-sequence="g1-steptouch-30s"][data-variant="between-ground-upright"]',
        )
        .waitFor();
      await app.locator(".configuration-table tbody tr").nth(1).waitFor();
      await page.waitForFunction(
        () =>
          document
            .querySelector("iframe")
            .contentDocument.querySelectorAll(".configuration-table tbody tr")
            .length === 4,
      );
      assert(
        await app
          .getByRole("checkbox", { name: "Weak motion prior" })
          .isDisabled(),
      );
      assert(
        await app
          .getByRole("checkbox", { name: "Weak motion prior" })
          .isChecked(),
      );
      assert.match(await app.locator(".notice").innerText(), /Disconnected/);
      await app.getByRole("checkbox", { name: "Sole ground priors" }).uncheck();
      await app.locator('.lab[data-variant="between-upright"]').waitFor();
      await app
        .getByRole("checkbox", { name: "Pelvis upright prior" })
        .uncheck();
      await app.locator('.lab[data-variant="between"]').waitFor();
      await app.getByRole("checkbox", { name: "Sole ground priors" }).check();
      await app.getByRole("checkbox", { name: "Pelvis upright prior" }).check();
      if (base === "/")
        await page.screenshot({
          path: "/tmp/g1-pose-contact-initializer.png",
          fullPage: true,
        });
      await page.setViewportSize({ width: 390, height: 844 });
      assert(
        await app
          .locator("body")
          .evaluate((b) => b.scrollWidth <= window.innerWidth + 1),
      );
      if (base === "/")
        await page.screenshot({
          path: "/tmp/g1-pose-contact-initializer-mobile.png",
          fullPage: true,
        });
      await page.getByRole("tab", { name: "Kinematics", exact: true }).click();
      await page
        .getByRole("tab", { name: "Contact Initialization", exact: true })
        .click();
      await app.locator('.scene[data-renderer="mujoco"]').waitFor();
      assert.deepEqual(errors, []);
      assert.deepEqual(failures, []);
      console.log(`Passed pose contact initialization ${base}`);
    } finally {
      await page.close();
      await new Promise((resolve) => server.httpServer.close(resolve));
    }
  }
} finally {
  await browser.close();
}
