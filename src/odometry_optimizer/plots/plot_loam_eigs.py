import matplotlib.pyplot as plt

degenerate_thresh = 30
bag_duration = 119.2 # seconds


def get_data():
    eigs_x = []
    eigs_y = []
    eigs_z = []
    eigs_roll = []
    eigs_pitch = []
    eigs_yaw = []
    with open("loam_eigs.txt") as f:
        for line in f.readlines():
            line_with_eigs = line.split("eigs: ")[1].split("<")[0]
            eigs = [float(eig_string) for eig_string in line_with_eigs.split()]

            eigs_x.append(eigs[0])
            eigs_y.append(eigs[1])
            eigs_z.append(eigs[2])

            eigs_roll.append(eigs[3])
            eigs_pitch.append(eigs[4])
            eigs_yaw.append(eigs[5])
    dt = bag_duration / len(eigs_x)
    ts = [dt * i for i in range(len(eigs_x))]
    return ts, eigs_x, eigs_y, eigs_z, eigs_roll, eigs_pitch, eigs_yaw

def plot():
    ts, eigs_x, eigs_y, eigs_z, eigs_roll, eigs_pitch, eigs_yaw = get_data()
    f = plt.figure(figsize=(7.2, 2.8))
    plt.xlabel("t [s]")
    plt.ylabel("eigenvalues of LOAM AtA matrix")
    for eigs, label in zip((eigs_x, eigs_y, eigs_z), (f"λ₁", "λ₂", "λ₃")):
        plt.plot(ts, eigs, label=label)
    plt.axline((ts[0], degenerate_thresh), (ts[-1], degenerate_thresh), label="degenerate thresh", color="m")
    plt.xlim(0, ts[-1])
    plt.legend()
    plt.savefig("loam_eigs.pdf", bbox_inches="tight")


    plt.figure()
    plt.xlabel("t [s]")
    plt.ylabel("eigenvalues of LOAM AtA matrix")
    for eigs, label in zip((eigs_roll, eigs_pitch, eigs_yaw), ("roll", "pitch", "yaw")):
        plt.plot(ts, eigs, label=label)
    plt.axline((ts[0], degenerate_thresh), (ts[-1], degenerate_thresh), label="degenerate thresh", color="m")
    plt.xlim(0, ts[-1])
    plt.legend()
    plt.savefig("loam_eigs_euler.pdf")

if __name__ == "__main__":
    plot()
