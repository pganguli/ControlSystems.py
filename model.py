import control as ct
import numpy as np


class Model:
    # Mass-Spring-Damper System
    @staticmethod
    def spring_mass(
        m: float = 1,  # mass (kg)
        k: float = 1,  # spring constant (N/m)
        b: float = 0.2,  # damping constant (Ns/m)
    ) -> ct.StateSpace:
        return ct.StateSpace(
            np.array([[0, 1], [-k / m, -b / m]]),  # A
            np.array([[0], [1 / m]]),  # B
            np.array([1, 0]),  # C
            0,  # D
            inputs=["F"],  # force
            outputs=["x"],  # position
            states=[
                "x",  # position
                "v",  # velocity
            ],
        )

    # RLC Circuit
    @staticmethod
    def rlc_circuit(
        R: float = 10,  # electric resistance (Ohm)
        L: float = 100,  # electric inductance (H)
        C: float = 0.01,  # electric capacitance (F)
    ) -> ct.StateSpace:
        return ct.StateSpace(
            np.array([[0, 1], [-1 / (L * C), -R / L]]),  # A
            np.array([[0], [1 / L]]),  # B
            np.array([1, 0]),  # C
            0,  # D
            inputs=["V"],  # voltage
            outputs=["q"],  # charge
            states=[
                "q",  # charge
                "i",  # current
            ],
        )

    # Cruise Control
    @staticmethod
    def cruise_control(
        m: float = 1000,  # vehicle mass (kg)
        b: float = 50,  # damping constant (N.s/m)
    ) -> ct.StateSpace:
        return ct.StateSpace(
            -b / m,  # A
            1 / m,  # B
            1,  # C
            0,  # D
            inputs=["F"],  # force
            outputs=["v"],  # velocity
            states=["v"],  # velocity
        )

    # DC Motor Position
    @staticmethod
    def dc_motor(
        J: float = 3.2284e-6,  # moment of inertia of the rotor (kg.m^2)
        b: float = 3.5077e-6,  # motor viscous friction constant (N.m.s)
        Kb: float = 0.0274,  # electromotive force constant (V/rad/sec)
        Kt: float = 0.0274,  # motor torque constant (N.m/A)
        R: float = 4,  # electric resistance (Ohm)
        L: float = 2.75e-6,  # electric inductance (H)
    ) -> ct.StateSpace:
        return ct.StateSpace(
            np.array([[0, 1, 0], [0, -b / J, -Kt / J], [0, -Kb / L, -R / L]]),  # A
            np.array([[0], [0], [1 / L]]),  # B
            np.array([1, 0, 0]),  # C
            0,  # D
            inputs=["V"],  # voltage
            outputs=["θ"],  # angle
            states=[
                "θ",  # angle
                "ω",  # angular velocity
                "i",  # current
            ],
        )

    # Inverted Pendulum
    @staticmethod
    def inv_pendulum(
        M: float = 0.5,  # mass of the cart (kg)
        m: float = 0.2,  # mass of the pendulum (kg)
        b: float = 0.1,  # coefficient of friction for cart (N/m/s)
        l: float = 0.3,  # length to pendulum center of mass (m)  # noqa: E741
        I: float = 0.006,  # moment of inertia of the pendulum (kg.m^2)  # noqa: E741
        g: float = 9.8,  # acceleration due to gravity (m/s^2)
    ) -> ct.StateSpace:
        p = I * (M + m) + M * m * (l**2)  # denominator for the A and B matrices
        return ct.StateSpace(
            np.array(
                [
                    [0, 1, 0, 0],
                    [0, -(I + m * (l**2)) * b / p, ((m**2) * g * (l**2)) / p, 0],
                    [0, 0, 0, 1],
                    [0, -(m * l * b) / p, m * g * l * (M + m) / p, 0],
                ]
            ),  # A
            np.array([[0], [(I + m * (l**2)) / p], [0], [m * l / p]]),  # B
            np.array([[1, 0, 0, 0], [0, 0, 1, 0]]),  # C
            0,  # D
            inputs=["F"],  # force
            outputs=["x", "φ"],  # position, angle
            states=[
                "x",  # position
                "v",  # velocity
                "φ",  # angle
                "ω",  # angular velocity
            ],
        )
