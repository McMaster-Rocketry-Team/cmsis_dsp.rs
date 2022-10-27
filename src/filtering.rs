use core::mem::MaybeUninit;
use fixed::types::{I1F15, I1F31};

macro_rules! make_fir_type {
    ($type: ty, $ptr_type: tt, $name: ident, $state: ident, $init_fn: ident, $run_fn: ident ) => {

        pub struct $name<'a, const NUM_TAPS: usize, const MAX_BLOCK_SIZE: usize>{
            instance: cmsis_dsp_sys::$state,
            _marker: core::marker::PhantomData<&'a ()>,
        }
        impl<'a, const NUM_TAPS: usize, const MAX_BLOCK_SIZE: usize>
        $name<'a, NUM_TAPS, MAX_BLOCK_SIZE>
        {
            pub fn new(
                coeffs: &'a [$type],
                state: &'a mut [$type]) -> Self
            {
                // When feature(generic_const_exprs) lands in stable the size can be contrained in
                // the argument type instead of runtime checking
                assert!(coeffs.len() >= NUM_TAPS);
                assert!(state.len() >= NUM_TAPS + MAX_BLOCK_SIZE - 1);

                // For q15 filter only, the provided coefficients must be an even number >= 4.
                check_valid_filter_size!($ptr_type, coeffs);
                let mut data = MaybeUninit::<cmsis_dsp_sys::$state>::uninit();

                unsafe {
                    cmsis_dsp_sys::$init_fn(
                        data.as_mut_ptr(),
                        NUM_TAPS as u16,
                        coeffs.as_ptr() as *const $ptr_type,
                        state.as_mut_ptr() as *mut $ptr_type,
                        MAX_BLOCK_SIZE as u32,
                    );
                    Self {instance: data.assume_init(), _marker: core::marker::PhantomData}
                }
            }

            pub fn run(&mut self, input: &[$type], output: &mut [$type]) {
                assert!(input.len() <= MAX_BLOCK_SIZE);
                assert!(output.len() >= input.len());

                unsafe {
                    cmsis_dsp_sys::$run_fn(
                        &self.instance,
                        input.as_ptr() as *const $ptr_type,
                        output.as_mut_ptr() as *mut $ptr_type,
                        input.len() as u32,
                    )
                }
            }
        }
    }
}

macro_rules! check_valid_filter_size {
    (f32, $c: ident) => {};
    (i32, $c: ident) => {};
    (i16, $c: ident) => {
        assert!($c.len() > 4);
        assert!(($c.len() % 2) == 0)
    };
}

macro_rules! make_fir_decimate_type {
    ($type: ty, $ptr_type: tt, $name: ident, $state: ident, $init_fn: ident, $run_fn: ident ) => {
        pub struct $name<'a, const NUM_TAPS: usize, const M: usize, const BLOCK_SIZE: usize>{
            instance: cmsis_dsp_sys::$state,
            _marker: core::marker::PhantomData<&'a ()>,
        }
        impl<'a, const NUM_TAPS: usize, const M: usize, const BLOCK_SIZE: usize>
        $name<'a, NUM_TAPS, M, BLOCK_SIZE>
        {
            pub fn new(
                coeffs: &'a [$type],
                state: &'a mut [$type]) -> Self
            {
                // When feature(generic_const_exprs) lands in stable the size can be contrained in
                // the argument type instead of runtime checking
                assert!(coeffs.len() >= NUM_TAPS);
                assert!(state.len() >= NUM_TAPS + BLOCK_SIZE - 1);
                assert!(BLOCK_SIZE % M == 0);

                let mut data = MaybeUninit::<cmsis_dsp_sys::$state>::uninit();

                unsafe {
                    let result = cmsis_dsp_sys::$init_fn(
                        data.as_mut_ptr(),
                        NUM_TAPS as u16,
                        M as u8,
                        coeffs.as_ptr() as *const $ptr_type,
                        state.as_mut_ptr() as *mut $ptr_type,
                        BLOCK_SIZE as u32,
                    );
                    if result != cmsis_dsp_sys::arm_status::ARM_MATH_SUCCESS {
                        panic!("Error initializing CMSIS DSP fir_decimate");
                    }
                    Self {instance: data.assume_init(), _marker: core::marker::PhantomData}
                }
            }

            pub fn run(&mut self, input: &[$type], output: &mut [$type]) {
                assert!(input.len() <= BLOCK_SIZE);
                assert!(output.len() >= input.len() / M);

                unsafe {
                    cmsis_dsp_sys::$run_fn(
                        &self.instance,
                        input.as_ptr() as *const $ptr_type,
                        output.as_mut_ptr() as *mut $ptr_type,
                        input.len() as u32,
                    )
                }
            }
        }
    }
}


make_fir_type!(f32, f32, FloatFir, arm_fir_instance_f32, arm_fir_init_f32, arm_fir_f32);
make_fir_type!(I1F15, i16, Q15Fir, arm_fir_instance_q15, arm_fir_init_q15, arm_fir_q15);
make_fir_type!(I1F31, i32, Q31Fir, arm_fir_instance_q31, arm_fir_init_q31, arm_fir_q31);

make_fir_decimate_type!(
    f32,
    f32,
    FloatFirDecimate,
    arm_fir_decimate_instance_f32,
    arm_fir_decimate_init_f32,
    arm_fir_decimate_f32
);
make_fir_decimate_type!(
    I1F15,
    i16,
    Q15FirDecimate,
    arm_fir_decimate_instance_q15,
    arm_fir_decimate_init_q15,
    arm_fir_decimate_q15
);
make_fir_decimate_type!(
    I1F31,
    i32,
    Q31FirDecimate,
    arm_fir_decimate_instance_q31,
    arm_fir_decimate_init_q31,
    arm_fir_decimate_q31
);