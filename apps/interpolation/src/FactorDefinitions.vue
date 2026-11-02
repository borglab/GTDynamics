<template>
  <section class="paper-section factor-definitions" aria-labelledby="factors-title">
    <h2 id="factors-title">Factors and fixed inputs</h2>
    <p>Each soft factor contributes ½ eᵀ Σ⁻¹ e. Let Δ = t<sub>k+1</sub> − t<sub>k</sub>,
      Q<sub>c</sub> be its diagonal acceleration PSD, and J<sub>r</sub> the SE(3) right Jacobian.
      Twists ν = [ω; v] are angular-first in the pelvis COM frame: ω is in rad/s and v in m/s.
      Joint angles q are in radians and joint rates r in rad/s. The definitions use the full
      configuration-and-velocity state; the drawing shows the reduced graph actually solved.</p>
    <article>
      <h3>Fixed knot configurations · exact substitution</h3>
      <p>At every selected slice, T<sub>k</sub> and q<sub>k</sub> equal the saved Kinematics
        solution. Their increments are removed from the Gaussian system. This is a hard condition,
        with no residual penalty, sigma, or additional adjustable pose. The upstream world gauge is retained.</p>
    </article>
    <article>
      <h3>G<sub>B</sub> · base WNOA process factor</h3>
      <p>Scope: (T<sub>k</sub>, ν<sub>k</sub>, T<sub>k+1</sub>, ν<sub>k+1</sub>),
        reduced to (ν<sub>k</sub>, ν<sub>k+1</sub>). The prior favors constant body twist in a
        local SE(3) chart, with white acceleration noise. Define ξ = Log(T<sub>k</sub>⁻¹ T<sub>k+1</sub>).</p>
      <div class="equation">e<sub>B</sub> = [ ξ − Δν<sub>k</sub> ;
        J<sub>r</sub>(ξ)⁻¹ν<sub>k+1</sub> − ν<sub>k</sub> ]</div>
      <p>The right-Jacobian transport compares endpoint velocities in a common local chart.
        Angular and linear PSDs use rad²/s³ and m²/s³, respectively.</p>
    </article>
    <article>
      <h3>G<sub>J</sub> · joint-vector WNOA process factor</h3>
      <p>Scope: (q<sub>k</sub>, r<sub>k</sub>, q<sub>k+1</sub>, r<sub>k+1</sub>),
        reduced to (r<sub>k</sub>, r<sub>k+1</sub>). One vector factor covers all joints.
        It favors constant joint rate with independent white acceleration noise per joint.</p>
      <div class="equation">e<sub>J</sub> = [ q<sub>k+1</sub> − q<sub>k</sub> − Δr<sub>k</sub> ;
        r<sub>k+1</sub> − r<sub>k</sub> ]</div>
      <p>Both process factors use the integrated WNOA covariance, including configuration–velocity correlation:</p>
      <div class="equation">Q(Δ) = [ (Δ³/3) Q<sub>c</sub>, (Δ²/2) Q<sub>c</sub> ;
        (Δ²/2) Q<sub>c</sub>, Δ Q<sub>c</sub> ]</div>
      <p>The joint PSD has units rad²/s³. Actual knot intervals determine Q(Δ), rather than a fixed
        sampling period. Accelerations are marginalized by the process model; they are not explicit unknowns.</p>
    </article>
    <article>
      <h3>M · dense interpolated encoder factor</h3>
      <p>For each encoder reading z(t) strictly inside a knot interval, the reduced scope is
        (r<sub>k</sub>, r<sub>k+1</sub>). Endpoint configurations are fixed inputs.
        With u = (t − t<sub>k</sub>)/Δ, the joint GP mean is cubic Hermite interpolation:</p>
      <div class="equation">q̄(t) = (2u³ − 3u² + 1) q<sub>k</sub> + (u³ − 2u² + u) Δr<sub>k</sub><br>
        + (−2u³ + 3u²) q<sub>k+1</sub> + (u³ − u²) Δr<sub>k+1</sub><br>
        e<sub>M</sub> = q̄(t) − z(t)</div>
      <p>The covariance combines encoder noise with the WNOA bridge's conditional position covariance.
        Let τ = t − t<sub>k</sub>:</p>
      <div class="equation">Σ<sub>M</sub>(t) = diag(σ<sub>enc</sub>²) +
        [ τ(Δ − τ)/Δ ]³ Q<sub>c,J</sub> / 3</div>
      <p>Residuals are in radians and covariance in rad². Bridge uncertainty grows away from the knots.
        Each timestamp contributes a separate vector factor; cross-time bridge correlations are omitted.
        Encoders at fixed knots cannot constrain velocities, so they are excluded from the solve but
        retained in the reported encoder RMS.</p>
    </article>
    <article>
      <h3>V · saved base world-linear velocity factor</h3>
      <p>Scope: ν<sub>k</sub> only. The measurement z<sub>v,k</sub> is the pelvis COM world-linear
        velocity saved by Kinematics. With R<sub>k</sub> the fixed COM-to-world rotation:</p>
      <div class="equation">e<sub>V</sub> = R<sub>k</sub> [0 I] ν<sub>k</sub> − z<sub>v,k</sub>,
        &nbsp; Σ<sub>V</sub> = σ<sub>v</sub>² I₃</div>
      <p>The residual is in world axes and m/s. This supplies no direct angular-velocity observation.
        The reusable API makes these measurements optional; the recording includes one at every knot.</p>
    </article>
    <p>These are all factor types in this solve. Contact, IMU, upright, height, and loop-closure factors
      belong to the upstream Kinematics graph; they are not added again here.</p>
  </section>
</template>
