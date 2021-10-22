import numpy as np


class RotationMatrix:
    def x_matrix(self, phi):
        sin = np.sin(phi)
        cos = np.cos(phi)

        if isinstance(phi, (int, float)):
            dim = 1
        else:
            dim = len(phi)

        mrx = np.zeros((dim, 3, 3))
        mrx[:, 0, 0] = np.ones(dim)
        mrx[:, 1, 1] = cos
        mrx[:, 2, 2] = cos
        mrx[:, 2, 1] = sin
        mrx[:, 1, 2] = -sin

        return mrx

    def y_matrix(self, phi):
        sin = np.sin(phi)
        cos = np.cos(phi)

        if isinstance(phi, (int, float)):
            dim = 1
        else:
            dim = len(phi)

        mrx = np.zeros((dim, 3, 3))
        mrx[:, 1, 1] = np.ones(dim)
        mrx[:, 0, 0] = cos
        mrx[:, 2, 2] = cos
        mrx[:, 0, 2] = sin
        mrx[:, 2, 0] = -sin

        return mrx

    def z_matrix(self, phi):
        sin = np.sin(phi)
        cos = np.cos(phi)

        if isinstance(phi, (int, float)):
            dim = 1
        else:
            dim = len(phi)

        mrx = np.zeros((dim, 3, 3))
        mrx[:, 2, 2] = np.ones(dim)
        mrx[:, 0, 0] = cos
        mrx[:, 1, 1] = cos
        mrx[:, 1, 0] = sin
        mrx[:, 0, 1] = -sin

        return mrx


class ScreenProjection:
    K1, K2 = 250, 400

    def project(self, fig):
        x = fig[:,0]
        y = fig[:,1]
        z = fig[:,2]

        self.x_ = 1.5*self.K1*x/(self.K2+z)
        self.y_ = self.K1*y/(self.K2+z)

        return self.x_, self.y_


class Donut:
    theta_pts = 200
    phi_pts   = 500
    A_step    = np.pi/200
    B_step    = 0

    lightning_vec = np.array([0, 1, -1])
    lightning_vec = lightning_vec/((lightning_vec**2).sum())**0.5
    mrx  = RotationMatrix()
    sprj = ScreenProjection()

    def __init__(self, R1, R2):
        self._R1 = R1
        self._R2 = R2

        self.thetas = np.linspace(0, 2*np.pi, self.theta_pts)
        self.phis   = np.linspace(0, 2*np.pi, self.phi_pts)

        rot = self.mrx.z_matrix(self.phis)

        circle = np.zeros((self.theta_pts, 3))
        circle[:, 0] = R2 + R1*np.cos(self.thetas)
        circle[:, 2] = R1*np.sin(self.thetas)

        norm = np.zeros((self.theta_pts, 3))
        norm[:, 0] = np.cos(self.thetas)
        norm[:, 2] = np.sin(self.thetas)

        _donut, _norm = [], []
        for dot, vec in zip(circle, norm):
            for mrx in rot:
                _donut.append(np.dot(dot, mrx))
                _norm.append(np.dot(vec, mrx))

        self._donut = np.array(_donut)
        self._norm  = np.array(_norm)

        self._loop()

    def _rotate_xz(self, A, B):
        x_mrx = self.mrx.y_matrix(A)
        z_mrx = self.mrx.z_matrix(B)

        idx = 0
        for dot, vec in zip(self._donut, self._norm):
            self._donut[idx] = np.dot(np.dot(dot, x_mrx), z_mrx)
            self._norm[idx] = np.dot(np.dot(vec, x_mrx), z_mrx)
            idx += 1

    def _loop(self):
        print((" "*120 + "\n")*50)
        I = 0
        while True:
            if I == 3:
                break
            I += 1
            self.z_buffer = {}
            self._rotate_xz(self.A_step, self.B_step)
            x, y = self.sprj.project(self._donut)

            L = []
            for norm in self._norm:
                L.append((self.lightning_vec*norm).sum())
            L = np.array(L)

            for x_c, y_c, z, L_c in zip (x, y, self._donut[:,2], L):
                x_c, y_c = int(x_c/10), int(y_c/10)

                coords = (60 - x_c, 25 - y_c)

                if L_c > 0:
                    L_c = ".,-~:;=!*#$@"[int(L_c * 12 - 0.001)]

                    if not coords in self.z_buffer:
                        self.z_buffer[coords] = (1/(self.sprj.K2+ z), L_c)
                    else:
                        if 1/(self.sprj.K2+ z) > self.z_buffer[coords][0]:
                            self.z_buffer[coords] = (1/(self.sprj.K2+ z), L_c)

            self.redraw()

    def redraw(self):
        template = np.full((50, 120), " ")

        print("\033[F"*(50))
        print((" "*120 + "\n")*47)
        print("\033[F"*(50))
        for key, val in self.z_buffer.items():
            if (key[1] >= 0 and key[1]) < 50 and (key[0] >= 0 and key[0] < 120):
                template[key[1], key[0]] = val[1]

        for row in template:
            print("".join(row))


if __name__ == "__main__":
    d = Donut(90, 180)
