// use numpy::{PyArray2, PyReadonlyArray2};
use pyo3::prelude::*;

#[pyfunction]
fn hello_from_bin() -> String {
    "Hello from sysco ".to_string()
}
// #[pyfunction]
// fn compute_discrete_step(
//     x: PyReadonlyArray2<f64>,
//     u: PyReadonlyArray2<f64>,
//     a: PyReadonlyArray2<f64>,
//     b: PyReadonlyArray2<f64>,
// ) -> PyResult<Py<PyArray2<f64>>> {
//     // Access the underlying data as ndarray (Rust's version of numpy)
//     let x = x.as_array();
//     let u = u.as_array();
//     let a = a.as_array();
//     let b = b.as_array();
//
//     // Perform the linear math: x_next = Ax + Bu
//     let x_next = a.dot(&x) + b.dot(&u);
//
//     // Convert back to a Python-managed NumPy array to return
//     Python::attach(|py| Ok(PyArray2::from_array(py, &x_next).to_owned()))
// }

/// A Python module implemented in Rust. The name of this function must match
/// the `lib.name` setting in the `Cargo.toml`, else Python will not be able to
/// import the module.
#[pymodule]
fn _core(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(hello_from_bin, m)?)?;
    // m.add_function(wrap_pyfunction!(compute_discrete_step, m)?)?;
    Ok(())
}
