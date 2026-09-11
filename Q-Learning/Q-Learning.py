"""Tabular Q-learning lab for a DJI RoboMaster EP/EP Core.

Run with --mode mock first. Real movement only starts with --mode robot.
The robot keeps its heading and uses the mecanum chassis to move one grid cell.
"""

from __future__ import annotations

import argparse
import csv
import json
import random
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path


if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")


ROWS, COLS = 4, 4
START, GOAL, PIT = (3, 0), (0, 3), (2, 2)
WALLS = {(1, 1), (2, 1)}
ACTIONS = ("UP", "DOWN", "LEFT", "RIGHT")
DELTA = {"UP": (-1, 0), "DOWN": (1, 0), "LEFT": (0, -1), "RIGHT": (0, 1)}
PERPENDICULAR = {"UP": ("LEFT", "RIGHT"), "DOWN": ("RIGHT", "LEFT"), "LEFT": ("DOWN", "UP"), "RIGHT": ("UP", "DOWN")}
INVERSE = {"UP": "DOWN", "DOWN": "UP", "LEFT": "RIGHT", "RIGHT": "LEFT"}
ARROW = {"UP": "↑", "DOWN": "↓", "LEFT": "←", "RIGHT": "→"}


@dataclass
class Transition:
    phase: str
    episode: int
    step: int
    state: str
    selected_action: str
    executed_action: str
    mode: str
    reward: float
    next_state: str
    terminal: bool
    epsilon: float
    old_q: float
    target: float
    td_error: float
    new_q: float


class MockController:
    def move(self, action: str) -> None:
        time.sleep(0.01)

    def stop(self) -> None:
        pass

    def close(self) -> None:
        pass


class RoboMasterController:
    """Small adapter around the official RoboMaster Python SDK."""

    def __init__(self, conn_type: str, cell_m: float, speed_mps: float) -> None:
        try:
            from robomaster import robot
        except ImportError as error:
            raise SystemExit("ไม่พบ RoboMaster SDK: ติดตั้งด้วย 'python -m pip install robomaster'") from error

        self.cell_m = cell_m
        self.speed_mps = speed_mps
        self.robot = robot.Robot()
        self.robot.initialize(conn_type=conn_type)
        self.chassis = self.robot.chassis

    def move(self, action: str) -> None:
        # DJI body axes: +x forward, -y left, +y right.
        x, y = {
            "UP": (self.cell_m, 0),
            "DOWN": (-self.cell_m, 0),
            "LEFT": (0, -self.cell_m),
            "RIGHT": (0, self.cell_m),
        }[action]
        self.chassis.move(x=x, y=y, z=0, xy_speed=self.speed_mps).wait_for_completed()

    def stop(self) -> None:
        self.chassis.drive_speed(x=0, y=0, z=0, timeout=1)

    def close(self) -> None:
        self.robot.close()


def state_key(state: tuple[int, int]) -> str:
    return f"{state[0]},{state[1]}"


def valid_state(state: tuple[int, int]) -> bool:
    row, col = state
    return 0 <= row < ROWS and 0 <= col < COLS and state not in WALLS


def next_state(state: tuple[int, int], action: str) -> tuple[int, int]:
    dr, dc = DELTA[action]
    candidate = (state[0] + dr, state[1] + dc)
    return candidate if valid_state(candidate) else state


def make_q_table() -> dict[tuple[int, int], dict[str, float]]:
    return {
        (row, col): {action: 0.0 for action in ACTIONS}
        for row in range(ROWS)
        for col in range(COLS)
        if valid_state((row, col))
    }


def greedy_actions(q_table: dict, state: tuple[int, int]) -> list[str]:
    best_value = max(q_table[state].values())
    return [action for action, value in q_table[state].items() if abs(value - best_value) < 1e-12]


def select_action(q_table: dict, state: tuple[int, int], epsilon: float, rng: random.Random) -> tuple[str, str]:
    """Epsilon-greedy behavior policy from Lecture 07."""
    if rng.random() < epsilon:
        return rng.choice(ACTIONS), "explore"
    return greedy_actions(q_table, state)[0], "exploit"


def q_update(q_table: dict, state: tuple[int, int], action: str, reward: float, successor: tuple[int, int], terminal: bool, alpha: float, gamma: float) -> tuple[float, float, float, float]:
    """Q(s,a) <- Q(s,a) + alpha [r + gamma max Q(s',.) - Q(s,a)]."""
    old_q = q_table[state][action]
    target = reward if terminal else reward + gamma * max(q_table[successor].values())
    td_error = target - old_q
    q_table[state][action] = old_q + alpha * td_error
    return old_q, target, td_error, q_table[state][action]


class MazeExperiment:
    def __init__(self, controller, q_table: dict, alpha: float, gamma: float, slip: float, max_steps: int, rng: random.Random, auto_reset: bool) -> None:
        self.controller = controller
        self.q_table = q_table
        self.alpha = alpha
        self.gamma = gamma
        self.slip = slip
        self.max_steps = max_steps
        self.rng = rng
        self.auto_reset = auto_reset
        self.records: list[Transition] = []

    def execute(self, state: tuple[int, int], selected_action: str) -> tuple[str, tuple[int, int], float, bool, bool]:
        executed_action = selected_action
        if self.rng.random() < self.slip:
            executed_action = self.rng.choice(PERPENDICULAR[selected_action])

        successor = next_state(state, executed_action)
        moved = successor != state
        if moved:
            self.controller.move(executed_action)

        terminal = successor in (GOAL, PIT)
        reward = 1.0 if successor == GOAL else -1.0 if successor == PIT else -0.20 if not moved else -0.04
        return executed_action, successor, reward, terminal, moved

    def reset_robot(self, physical_path: list[str]) -> None:
        if not self.auto_reset:
            input("ยก/วางหุ่นกลับจุด S แล้วกด Enter...")
            return
        for action in reversed(physical_path):
            self.controller.move(INVERSE[action])

    def run_episode(self, episode: int, epsilon: float, phase: str, learn: bool) -> tuple[float, bool, int]:
        state, physical_path, total_reward = START, [], 0.0
        for step in range(1, self.max_steps + 1):
            selected, mode = select_action(self.q_table, state, epsilon, self.rng)
            executed, successor, reward, terminal, moved = self.execute(state, selected)
            if moved:
                physical_path.append(executed)

            old_q = self.q_table[state][selected]
            if learn:
                old_q, target, td_error, new_q = q_update(
                    self.q_table, state, selected, reward, successor, terminal, self.alpha, self.gamma
                )
            else:
                target = reward if terminal else reward + self.gamma * max(self.q_table[successor].values())
                td_error, new_q = target - old_q, old_q

            self.records.append(
                Transition(
                    phase, episode, step, state_key(state), selected, executed, mode,
                    reward, state_key(successor), terminal, epsilon,
                    old_q, target, td_error, new_q,
                )
            )
            total_reward += reward
            state = successor
            if terminal:
                break

        self.reset_robot(physical_path)
        return total_reward, state == GOAL, step


def epsilon_at(episode: int, total: int, initial: float, final: float) -> float:
    return final if total <= 1 else initial + (final - initial) * (episode - 1) / (total - 1)


def print_policy(q_table: dict) -> None:
    print("\nGreedy policy")
    for row in range(ROWS):
        cells = []
        for col in range(COLS):
            state = (row, col)
            cells.append("▧" if state in WALLS else "G" if state == GOAL else "P" if state == PIT else "S" if state == START else ARROW[greedy_actions(q_table, state)[0]])
        print("  ".join(cells))


def save_results(output_dir: Path, q_table: dict, records: list[Transition], summary: dict) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    with (output_dir / "transitions.csv").open("w", newline="", encoding="utf-8-sig") as file:
        writer = csv.DictWriter(file, fieldnames=Transition.__dataclass_fields__)
        writer.writeheader()
        writer.writerows(asdict(record) for record in records)
    (output_dir / "q-table.json").write_text(
        json.dumps({state_key(state): values for state, values in q_table.items()}, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    (output_dir / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Week 05: Q-learning with DJI RoboMaster EP/EP Core")
    parser.add_argument("--mode", choices=("mock", "robot"), default="mock")
    parser.add_argument("--conn-type", choices=("ap", "sta", "rndis"), default="ap")
    parser.add_argument("--episodes", type=int, default=60)
    parser.add_argument("--evaluate", type=int, default=5)
    parser.add_argument("--epsilon", type=float, default=0.40)
    parser.add_argument("--epsilon-final", type=float, default=0.05)
    parser.add_argument("--alpha", type=float, default=0.50)
    parser.add_argument("--gamma", type=float, default=0.90)
    parser.add_argument("--slip", type=float, default=0.15)
    parser.add_argument("--cell-m", type=float, default=0.35)
    parser.add_argument("--speed-mps", type=float, default=0.50)
    parser.add_argument("--max-steps", type=int, default=25)
    parser.add_argument("--calibrate-every", type=int, default=10)
    parser.add_argument("--motion-check", action="store_true")
    parser.add_argument("--manual-reset", action="store_true")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--output-dir", type=Path, default=Path("lab-output"))
    args = parser.parse_args()
    for name in ("epsilon", "epsilon_final", "alpha", "gamma", "slip"):
        if not 0 <= getattr(args, name) <= 1:
            parser.error(f"--{name.replace('_', '-')} ต้องอยู่ระหว่าง 0 และ 1")
    if args.episodes < 1 or args.evaluate < 0 or args.max_steps < 1:
        parser.error("episodes/max-steps ต้องเป็นค่าบวก และ evaluate ต้องไม่ติดลบ")
    if args.mode == "robot" and args.speed_mps < 0.5:
        parser.error("RoboMaster chassis.move กำหนด xy_speed ขั้นต่ำ 0.5 m/s")
    return args


def main() -> None:
    args = parse_args()
    rng = random.Random(args.seed)
    controller = MockController() if args.mode == "mock" else RoboMasterController(args.conn_type, args.cell_m, args.speed_mps)
    if args.motion_check:
        if args.mode == "robot":
            input("ตรวจพื้นที่ว่างรอบหุ่นอย่างน้อย 1 เมตร แล้วกด Enter เพื่อวิ่งรูปสี่เหลี่ยม...")
        try:
            for action in ("UP", "RIGHT", "DOWN", "LEFT"):
                print("motion check:", action)
                controller.move(action)
        finally:
            try:
                controller.stop()
            finally:
                controller.close()
        return

    experiment = MazeExperiment(controller, make_q_table(), args.alpha, args.gamma, args.slip, args.max_steps, rng, not args.manual_reset)
    train_results, eval_results = [], []

    print("เริ่มในโหมด", args.mode)
    print("ผัง: S=(3,0), G=(0,3), P=(2,2), walls=(1,1),(2,1)")
    if args.mode == "robot":
        input("ตรวจพื้นที่ วางหุ่นกึ่งกลางช่อง S ให้ด้านหน้าหันไปทาง UP แล้วกด Enter...")

    try:
        for episode in range(1, args.episodes + 1):
            epsilon = epsilon_at(episode, args.episodes, args.epsilon, args.epsilon_final)
            total, success, steps = experiment.run_episode(episode, epsilon, "train", True)
            train_results.append((total, success, steps))
            print(f"train {episode:02d} | ε={epsilon:.2f} | return={total:+.2f} | steps={steps:02d} | {'GOAL' if success else 'MISS'}")
            if args.mode == "robot" and args.calibrate_every and episode % args.calibrate_every == 0 and episode < args.episodes:
                input("ตรวจและจัดหุ่นให้อยู่กึ่งกลาง S อีกครั้ง แล้วกด Enter...")

        for episode in range(1, args.evaluate + 1):
            total, success, steps = experiment.run_episode(episode, 0.0, "evaluate", False)
            eval_results.append((total, success, steps))
            print(f"eval  {episode:02d} | ε=0.00 | return={total:+.2f} | steps={steps:02d} | {'GOAL' if success else 'MISS'}")
    except KeyboardInterrupt:
        print("\nหยุดโดยผู้ใช้")
    finally:
        try:
            controller.stop()
        finally:
            controller.close()

    visited = {(record.state, record.selected_action) for record in experiment.records if record.phase == "train"}
    available_pairs = sum(state not in (GOAL, PIT) for state in experiment.q_table) * len(ACTIONS)
    parameters = vars(args).copy()
    parameters["output_dir"] = str(args.output_dir)
    summary = {
        "mode": args.mode,
        "seed": args.seed,
        "train_episodes_completed": len(train_results),
        "train_success_rate": sum(success for _, success, _ in train_results) / len(train_results) if train_results else 0,
        "train_mean_return": sum(total for total, _, _ in train_results) / len(train_results) if train_results else 0,
        "first_goal_episode": next((index for index, (_, success, _) in enumerate(train_results, 1) if success), None),
        "evaluation_episodes_completed": len(eval_results),
        "evaluation_success_rate": sum(success for _, success, _ in eval_results) / len(eval_results) if eval_results else 0,
        "evaluation_mean_steps": sum(steps for _, _, steps in eval_results) / len(eval_results) if eval_results else 0,
        "state_action_coverage": len(visited) / available_pairs,
        "unique_state_action_pairs": len(visited),
        "available_state_action_pairs": available_pairs,
        "slip_transition_count": sum(record.selected_action != record.executed_action for record in experiment.records if record.phase == "train"),
        "parameters": parameters,
    }
    save_results(args.output_dir, experiment.q_table, experiment.records, summary)
    print_policy(experiment.q_table)
    print(f"\ncoverage={summary['state_action_coverage']:.1%} | eval success={summary['evaluation_success_rate']:.1%}")
    print("บันทึกผลที่", args.output_dir.resolve())


if __name__ == "__main__":
    main()
