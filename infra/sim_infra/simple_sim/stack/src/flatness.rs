//! Safe fixed-dimension wrapper around the generated, reentrant C kernel.
#[derive(Debug, PartialEq)]
pub struct Derivatives {
    /// Tangent acceleration, omega_sx, omega_by, omega_sz (not thrust magnitude).
    pub value: [f64; 4],
    /// J[row + 4 * column].
    pub jacobian: [f64; 36],
    /// J^T times the caller's cotangent seed.
    pub backward: [f64; 9],
}

unsafe extern "C" {
    fn ap_pnc_flatness_flu(
        z: *const f64,
        params: *const f64,
        seed: *const f64,
        value: *mut f64,
        jacobian: *mut f64,
        gradient: *mut f64,
    ) -> i32;
    fn ap_pnc_flatness(
        z: *const f64,
        params: *const f64,
        seed: *const f64,
        value: *mut f64,
        jacobian: *mut f64,
        backward: *mut f64,
    ) -> i32;
    fn ap_pnc_flatness_backward(
        z: *const f64,
        params: *const f64,
        seed: *const f64,
        value: *mut f64,
        gradient: *mut f64,
    ) -> i32;
}

#[derive(Debug, PartialEq)]
pub struct Backward {
    pub value: [f64; 4],
    pub gradient: [f64; 9],
}

/// Direct VJP entry; does not materialize the 4x9 Jacobian.
pub fn backward(z: &[f64; 9], seed: &[f64; 4]) -> Result<Backward, &'static str> {
    if !z.iter().chain(seed).all(|x| x.is_finite()) {
        return Err("nonfinite input");
    }
    let params = [-9.8, -0.09408, 0.5, 0.05];
    let mut out = Backward {
        value: [0.0; 4],
        gradient: [0.0; 9],
    };
    // SAFETY: fixed-length, live, non-aliasing buffers; C keeps no pointers and
    // uses only local workspace, as for the diagnostic full-Jacobian entry.
    let status = unsafe {
        ap_pnc_flatness_backward(
            z.as_ptr(),
            params.as_ptr(),
            seed.as_ptr(),
            out.value.as_mut_ptr(),
            out.gradient.as_mut_ptr(),
        )
    };
    if status != 0 {
        return Err("C kernel failure");
    }
    if !out.value.iter().chain(&out.gradient).all(|x| x.is_finite()) {
        return Err("nonfinite output");
    }
    Ok(out)
}

/// Branchwise optimization-map derivatives, using the exact legacy constants.
/// Classical differentiability is NOT promised on either switching surface.
pub fn evaluate(z: &[f64; 9], seed: &[f64; 4]) -> Result<Derivatives, &'static str> {
    if !z.iter().chain(seed).all(|x| x.is_finite()) {
        return Err("nonfinite input");
    }
    let params = [-9.8, -0.09408, 0.5, 0.05];
    let mut out = Derivatives {
        value: [0.0; 4],
        jacobian: [0.0; 36],
        backward: [0.0; 9],
    };
    // SAFETY: all six buffers have the ABI's exact lengths, remain alive for
    // the call, and do not alias. The generated C wrapper uses stack workspaces,
    // retains no pointers, and has no shared mutable state.
    let status = unsafe {
        ap_pnc_flatness(
            z.as_ptr(),
            params.as_ptr(),
            seed.as_ptr(),
            out.value.as_mut_ptr(),
            out.jacobian.as_mut_ptr(),
            out.backward.as_mut_ptr(),
        )
    };
    if status != 0 {
        return Err("C kernel failure");
    }
    if !out
        .value
        .iter()
        .chain(&out.jacobian)
        .chain(&out.backward)
        .all(|x| x.is_finite())
    {
        return Err("nonfinite output");
    }
    Ok(out)
}

/// Direct main-frame output: collective N/kg, omega_FLU xyz, R_WB row-major.
#[derive(Debug, PartialEq)]
pub struct FluDerivatives {
    pub value: [f64; 13],
    /// J[row + 13*column], inputs v/a/jerk in ENU.
    pub jacobian: [f64; 117],
    pub backward: [f64; 9],
}

pub fn evaluate_flu(
    z: &[f64; 9],
    params: &[f64; 4],
    seed: &[f64; 13],
) -> Result<FluDerivatives, &'static str> {
    if !z.iter().chain(params).chain(seed).all(|x| x.is_finite())
        || params[0] >= 0.0
        || params[2] <= 0.0
        || params[3] <= 0.0
    {
        return Err("invalid FLU input/parameters");
    }
    let mut out = FluDerivatives {
        value: [0.0; 13],
        jacobian: [0.0; 117],
        backward: [0.0; 9],
    };
    // SAFETY: exact fixed ABI lengths, disjoint live buffers, no retained pointers/shared workspace.
    let status = unsafe {
        ap_pnc_flatness_flu(
            z.as_ptr(),
            params.as_ptr(),
            seed.as_ptr(),
            out.value.as_mut_ptr(),
            out.jacobian.as_mut_ptr(),
            out.backward.as_mut_ptr(),
        )
    };
    if status != 0
        || !out
            .value
            .iter()
            .chain(&out.jacobian)
            .chain(&out.backward)
            .all(|x| x.is_finite())
    {
        return Err("FLU kernel failed/nonfinite");
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn flu_hover_and_vjp() {
        let params = [-9.81, -0.09408, 0.5, 0.05];
        let seed = [1.0; 13];
        let hover = evaluate_flu(&[0.0; 9], &params, &seed).unwrap();
        assert_eq!(&hover.value[..4], &[9.81, 0.0, 0.0, 0.0]);
        assert_eq!(
            &hover.value[4..],
            &[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        );
        let sample = evaluate_flu(
            &[6.0, 1.0, 0.2, 0.1, 0.2, 0.3, 0.3, 0.2, 0.1],
            &params,
            &seed,
        )
        .unwrap();
        for col in 0..9 {
            let expected: f64 = (0..13).map(|row| sample.jacobian[row + 13 * col]).sum();
            assert!((expected - sample.backward[col]).abs() < 1e-10);
        }
        assert!(evaluate_flu(&[f64::NAN; 9], &params, &seed).is_err());
        assert!(evaluate_flu(&[0.0; 9], &[-9.81, 0.0, 0.0, 0.05], &seed).is_err());
    }
    #[test]
    fn zero_speed_is_finite() {
        let r = evaluate(&[0.0; 9], &[1.0; 4]).unwrap();
        assert_eq!(r.value, [0.0; 4]);
        assert_eq!(r.jacobian, [0.0; 36]);
        assert_eq!(r.backward, [0.0; 9]);
    }
    #[test]
    fn projected_degenerate_policy() {
        let r = evaluate(&[5.0, 0.0, 0.0, 1.0, 0.0, -9.8, 0.2, 0.1, 0.0], &[1.0; 4]).unwrap();
        assert_eq!(r.value, [1.0, 0.0, 0.0, 0.0]);
    }
    #[test]
    fn backward_layout_and_concurrency() {
        let z = [8.0, 1.0, 0.3, 0.2, -0.1, 0.4, 0.1, 0.3, -0.2];
        let seed = [0.5, -1.0, 2.0, 0.7];
        let expected = evaluate(&z, &seed).unwrap();
        for col in 0..9 {
            let sum: f64 = seed
                .iter()
                .enumerate()
                .map(|(row, s)| s * expected.jacobian[row + 4 * col])
                .sum();
            assert!((sum - expected.backward[col]).abs() < 1e-12);
        }
        std::thread::scope(|scope| {
            for _ in 0..8 {
                let e = &expected;
                scope.spawn(move || {
                    for _ in 0..100 {
                        assert_eq!(evaluate(&z, &seed).unwrap(), *e);
                    }
                });
            }
        });
    }
    #[test]
    fn direct_vjp_matches_full_jacobian() {
        let z = [8.0, 1.0, 0.3, 0.2, -0.1, 0.4, 0.1, 0.3, -0.2];
        let seed = [0.5, -1.0, 2.0, 0.7];
        let full = evaluate(&z, &seed).unwrap();
        let direct = backward(&z, &seed).unwrap();
        for (a, b) in direct.value.iter().zip(full.value) {
            assert!((a - b).abs() < 1e-12);
        }
        for (a, b) in direct.gradient.iter().zip(full.backward) {
            assert!((a - b).abs() < 1e-12);
        }
        assert_eq!(backward(&[0.0; 9], &seed).unwrap().gradient, [0.0; 9]);
        assert!(backward(&[f64::NAN; 9], &seed).is_err());
    }
    #[test]
    fn rejects_nan() {
        assert!(evaluate(&[f64::NAN; 9], &[1.0; 4]).is_err());
    }
}
