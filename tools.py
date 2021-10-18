import numpy as np
import pygame


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
        x = fig[:, 0]
        y = fig[:, 1]
        z = fig[:, 2]

        self.x_ = self.K1*x/(self.K2+z)
        self.y_ = self.K1*y/(self.K2+z)

        return self.x_, self.y_


class Donut:
    theta_pts = 50
    phi_pts   = 50
    A_step    = np.pi/111
    B_step    = np.pi/213

    lightning_vec = np.array([0, 1/2**0.5, -1/2**0.5])
    mrx  = RotationMatrix()
    sprj = ScreenProjection()

    def __init__(self, R1, R2):
        self._R1 = R1
        self._R2 = R2

        self.thetas = np.linspace(0, 2*np.pi, self.theta_pts)
        self.phis   = np.linspace(0, 2*np.pi, self.phi_pts)

        y_rot = self.mrx.y_matrix(self.phis)

        circle = np.zeros((self.theta_pts, 3))
        circle[:, 0] = R2 + R1*np.cos(self.thetas)
        circle[:, 1] = R1*np.sin(self.thetas)

        norm = np.zeros((self.theta_pts, 3))
        norm[:, 0] = np.cos(self.thetas)
        norm[:, 1] = np.sin(self.thetas)

        _donut, _norm = [], []
        for dot, vec in zip(circle, norm):
            for mrx in y_rot:
                _donut.append(np.dot(dot, mrx))
                _norm.append(np.dot(vec, mrx))

        self._donut = np.array(_donut)
        self._norm  = np.array(_norm)

        self._loop()

    def _rotate_xz(self, A, B):
        x_mrx = self.mrx.x_matrix(A)
        z_mrx = self.mrx.z_matrix(B)

        idx = 0
        for dot, vec in zip(self._donut, self._norm):
            self._donut[idx] = np.dot(np.dot(dot, x_mrx), z_mrx)
            self._norm[idx] = np.dot(np.dot(vec, x_mrx), z_mrx)
            idx += 1

    def _loop(self):
        while True:
            pygame.time.delay(int(10))
            self._rotate_xz(self.A_step, self.B_step)
            x, y = self.sprj.project(self._donut)
            window.fill((0, 0, 0))

            L = []
            for norm in self._norm:
                L.append((self.lightning_vec*norm).sum())
            L = np.array(L)

            for x_c, y_c, L_c in zip (x, y, L):
                if L_c > 0:
                    L_c = int(L_c * 255)
                    pygame.draw.circle(window, (L_c, L_c, L_c), (x_c + 400, 400 - y_c), 1)

            pygame.display.update()

if __name__ == "__main__":
    pygame.init()
    window = pygame.display.set_mode((800, 800))
    d = Donut(100, 200)




