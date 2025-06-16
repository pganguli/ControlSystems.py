import numpy as np
import control as ct
import matplotlib.pyplot as plt

from model import Model


def controller(sys: ct.StateSpace) -> tuple[np.ndarray, np.ndarray]:
    """Compute the LQR gain matrix and feedforward term for the given system."""
    K, _, _ = ct.dlqr(
        sys,
        np.eye(sys.nstates),  # State cost matrix Q
        np.eye(sys.ninputs),  # Input cost matrix R
    )
    # Feedforward term for reference tracking, as a row vector
    F = np.array(
        1 / (sys.C @ np.linalg.inv(np.eye(sys.nstates) - sys.A + sys.B @ K) @ sys.B)
    ).reshape(1, -1)
    return K, F


def simulate(
    sys: ct.StateSpace,
    t: np.ndarray,
    K: np.ndarray,
    F: np.ndarray,
    x0: np.ndarray,
    y_ref: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Initialize and simulate the closed-loop system with LQR controller."""
    x = np.zeros((len(t), sys.nstates, 1))
    u = np.zeros((len(t), sys.ninputs, 1))
    x[0] = x0
    u[0] = -K @ x0 + F @ y_ref
    for k in range(1, len(t)):
        x[k] = sys.A @ x[k - 1] + sys.B @ u[k - 1]
        u[k] = -K @ x[k] + F @ y_ref
    return x, u


def plot_sim(sys: ct.StateSpace, t: np.ndarray, x: np.ndarray, u: np.ndarray) -> None:
    """Plot the states and inputs over time."""
    _, axs = plt.subplots(2, 1, figsize=(8, 8), sharex=True)

    # Plot all states
    for i in range(sys.nstates):
        axs[0].plot(t, x[:, i], label=sys.state_labels[i])
    axs[0].set_ylabel("States")
    axs[0].legend()

    # Plot all inputs
    for j in range(sys.ninputs):
        axs[1].plot(t, u[:, j], label=sys.input_labels[j])
    axs[1].set_ylabel("Inputs")
    axs[1].set_xlabel("Time (s)")
    axs[1].legend()

    plt.tight_layout()
    plt.show()


def report_stats(
    t: np.ndarray,
    x: np.ndarray,
    u: np.ndarray,
    y_ref: np.ndarray,
    sys: ct.StateSpace,
    tol: float = 0.01,
) -> None:
    """Report time to converge to reference and max control input exerted."""
    print("\n--- Simulation Statistics ---")
    try:
        y = np.atleast_2d(np.matmul(sys.C, x[..., 0].T).T)
        y_target = y_ref.flatten()
        within_tol = np.all(np.abs(y - y_target) < tol * np.abs(y_target), axis=1)
        converge_idx = np.where(within_tol & np.cumprod(within_tol[::-1])[::-1])[0][0]
        converge_time = t[converge_idx]
        print(
            f"Time to converge to reference (within {tol*100:.1f}%): {converge_time:.2f} s"
        )
    except IndexError:
        print(f"Did not converge to reference within {tol*100:.1f}% tolerance.")
    max_u = np.max(np.abs(u))
    print(f"Maximum control input exerted: {max_u:.3f}")


def main() -> None:
    """Main function to run the simulation."""
    T = 200.0
    dt = 0.1
    t = np.arange(0, T + dt, dt)

    sys = Model.rlc_circuit()
    x0 = np.array([[50], [0]])
    y_ref = np.array([[10]])

    sys_d = ct.c2d(sys, dt)
    K, F = controller(sys_d)
    print(K)
    print(F)
    x, u = simulate(sys_d, t, K, F, x0, y_ref)
    plot_sim(sys_d, t, x, u)
    report_stats(t, x, u, y_ref, sys_d)


if __name__ == "__main__":
    main()
