import numpy as np

k_arr = [1, 2, 3, 4, 5]
w_arr = [0, 1, 2, 1, 0]
u_allowable = [0, 1, 2]
h_allowable = [8, 7, 6, 5, 4, 3]
hf_allowable = [8, 7, 6]
N = len(k_arr)

h_optimal_arr = np.zeros_like(k_arr)
u_optimal_arr = np.zeros_like(k_arr)
J_optimal_arr = np.zeros((len(h_allowable), N))

for k in reversed(range(N)):
    benefits = {}
    w = w_arr[k]
    for h in h_allowable:
        J_max = -np.inf
        for u in u_allowable:
            h_next = h + w - u
            if k == 4:
                # First iteration
                if h_next in hf_allowable:
                    J = 0.1 * h * u - 0.5 * (h_next - 8) ** 2
                    if J > J_max:
                        # If J is maximized for current state
                        u_optimal = u
                        u_optimal_arr[k] = u_optimal
                        J_max = J
                        benefits.update({h: J_max})
            else:
                # Not first iteration
                if h_next in h_allowable:
                    J = 0.1 * h * u + benefits_next[h_next]
                    if J > J_max:
                        # If J is maximized for current state
                        u_optimal = u
                        u_optimal_arr[k] = u_optimal
                        J_max = J
                        benefits.update({h: J_max})
    benefits_next = benefits
    print(benefits)
