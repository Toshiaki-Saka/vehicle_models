# 運動方程式

English edition: [`docs_en/models.md`](../docs_en/models.md)

記号： $L = l_f + l_r$（ホイールベース）、 $m$（質量）、 $I_z$（ヨー慣性）、 $C_f, C_r$（軸あたりコーナリングスティフネス $[\mathrm{N/rad}]$）、 $\delta$（前輪舵角）、 $\psi$（ヨー角）、 $r = \dot\psi$（ヨーレート）、 $\beta$（車体スリップ角）。座標は右手系、 $\delta > 0$ が左旋回。

---

## 1. ユニサイクル / 差動二輪

$$
\begin{aligned}
\dot x &= v \cos\psi \\
\dot y &= v \sin\psi \\
\dot\psi &= \omega
\end{aligned}
$$

差動二輪では車輪角速度 $\omega_l, \omega_r$、車輪半径 $R_w$、トレッド $b$ から

$$
v = \frac{R_w (\omega_r + \omega_l)}{2},
\qquad
\omega = \frac{R_w (\omega_r - \omega_l)}{b}
$$

旋回半径に下限がない（その場旋回可能）ため、アッカーマン操舵車両の挙動モデルとしては誤りです。プランナ側の抽象、またはスキッドステア機に限定して使ってください。

---

## 2. アッカーマン幾何

後軸中心の旋回半径 $R = L / \tan\delta$。理想アッカーマンは

$$
\cot\delta_{\text{outer}} - \cot\delta_{\text{inner}} = \frac{T}{L}
$$

を満たします（ $T$ はフロントトレッド）。本ライブラリは

$$
\cot\delta_{\text{left}} = \cot\delta - \frac{T}{2L},
\qquad
\cot\delta_{\text{right}} = \cot\delta + \frac{T}{2L}
$$

で理想値を求め、`ackermann_ratio` $k$ により平行操舵との線形ブレンドを取ります。

$$
\delta_i = \delta + k\ (\delta_{i,\text{ideal}} - \delta)
\qquad
\begin{cases}
k = 1 & \text{理想アッカーマン} \\
k = 0 & \text{平行操舵}
\end{cases}
$$

実車のラックは $k < 1$（アッカーマン率 60〜80 % など）が普通で、`ackermannError()` はその外輪側のずれを返します。

車輪速（剛体運動）：

$$
\begin{aligned}
v_{rl} &= v - \frac{T_r}{2} r,
&\qquad
v_{rr} &= v + \frac{T_r}{2} r \\
v_{fl} &= \left(v - \frac{T_f}{2} r\right)\cos\delta_l + L r \sin\delta_l,
&\qquad
v_{fr} &= \left(v + \frac{T_f}{2} r\right)\cos\delta_r + L r \sin\delta_r
\end{aligned}
$$

前輪は操舵方向へ射影しており、車輪速センサが観測する量に対応します。

---

## 3. キネマティック二輪

タイヤ横すべりゼロを仮定。基準点によって式が変わります。

**後軸基準**

$$
\dot x = v\cos\psi,
\qquad
\dot y = v\sin\psi,
\qquad
\dot\psi = \frac{v\tan\delta}{L},
\qquad
\dot v = a
$$

**重心基準**（ $\beta = \arctan\left(l_r \tan\delta / L\right)$、 $v$ は重心速度）

$$
\dot x = v\cos(\psi + \beta),
\qquad
\dot y = v\sin(\psi + \beta),
\qquad
\dot\psi = \frac{v\cos\beta\ \tan\delta}{L}
$$

**前軸基準**（ $v$ は前輪速度）

$$
\dot x = v\cos(\psi + \delta),
\qquad
\dot y = v\sin(\psi + \delta),
\qquad
\dot\psi = \frac{v\sin\delta}{L}
$$

妥当範囲：横加速度が概ね $0.4\ g$ 以下。それ以上ではタイヤのスリップ角が無視できず、ヨーレートを大きく過大評価します（`examples/step_steer.cpp`、および Python GUI の「操縦」タブが同じ比較を対話的に示します）。

操舵アクチュエータを含む版：

$$
\dot\delta = \mathrm{clamp}\left(\frac{\delta_{\text{cmd}} - \delta}{\tau},\ \pm\dot\delta_{\max}\right)
$$

---

## 4. 線形2自由度（横運動）モデル

$v_x$ 一定、微小角を仮定した古典的ハンドリングモデル。

$$
\frac{d}{dt}
\begin{bmatrix}
v_y \\
r
\end{bmatrix}
=
\begin{bmatrix}
-\dfrac{C_f + C_r}{m v_x} & -v_x - \dfrac{l_f C_f - l_r C_r}{m v_x} \\
-\dfrac{l_f C_f - l_r C_r}{I_z v_x} & -\dfrac{l_f^2 C_f + l_r^2 C_r}{I_z v_x}
\end{bmatrix}
\begin{bmatrix}
v_y \\
r
\end{bmatrix} +
\begin{bmatrix}
\dfrac{C_f}{m} \\
\dfrac{l_f C_f}{I_z}
\end{bmatrix}
\delta
$$

`stateMatrix()` / `inputMatrix()` がこの $A, B$ をそのまま返すので、LQR や MPC の設計プラント、あるいはオブザーバの予測モデルに直接使えます。

### 解析解（`linear_analysis.hpp`）

アンダーステア勾配：

$$
K = \frac{m}{L}\left(\frac{l_r}{C_f} - \frac{l_f}{C_r}\right)
\quad [\mathrm{rad/(m/s^2)}],
\qquad
\delta = \frac{L}{R} + K a_y
$$

- $K > 0$：アンダーステア → 特性速度 $V_{ch} = \sqrt{L/K}$（ヨーレートゲインがニュートラルステア時の半分になる速度）
- $K < 0$：オーバーステア → 限界速度 $V_{cr} = \sqrt{-L/K}$（これを超えると発散）

定常円旋回：

$$
\frac{r}{\delta} = \frac{v}{L + K v^2},
\qquad
\frac{a_y}{\delta} = \frac{v^2}{L + K v^2},
\qquad
\beta = \frac{1}{R}\left(l_r - \frac{m l_f v^2}{L C_r}\right)
$$

ニュートラルステアポイント（前軸からの距離）と スタティックマージン：

$$
x_{NSP} = \frac{L C_r}{C_f + C_r},
\qquad
SM = \frac{x_{NSP} - l_f}{L}
$$

ヨーモードは $A$ の固有値から

$$
\omega_n = \sqrt{\det A},
\qquad
\zeta = -\frac{\mathrm{tr} A}{2\sqrt{\det A}}
$$

---

## 5. 非線形単輪（ダイナミック二輪）

$$
\begin{aligned}
m\left(\dot v_x - v_y r\right) &= F_x - F_{yf}\sin\delta - F_{\text{res}} \\
m\left(\dot v_y + v_x r\right) &= F_{yf}\cos\delta + F_{yr} \\
I_z \dot r &= l_f F_{yf}\cos\delta - l_r F_{yr}
\end{aligned}
$$

スリップ角：

$$
\alpha_f = \delta - \arctan\frac{v_y + l_f r}{v_x},
\qquad
\alpha_r = -\arctan\frac{v_y - l_r r}{v_x}
$$

$v_x$ はガード付き（`guardDenominator`）。前後荷重移動は準静的に

$$
\Delta F_z = \frac{m a_x h}{L},
\qquad
F_{zf} = \frac{m g l_r}{L} - \Delta F_z,
\qquad
F_{zr} = \frac{m g l_f}{L} + \Delta F_z
$$

走行抵抗は空気抵抗と転がり抵抗の和：

$$
F_{\text{res}} = \underbrace{\tfrac{1}{2}\rho C_d A\ v_x |v_x|}_{\text{空気抵抗}} +
\underbrace{\mu_{rr}\ m g \tanh\frac{v_x}{0.1}}_{\text{転がり抵抗}}
$$

第1項の係数 $\tfrac{1}{2}\rho C_d A$ がパラメータ `drag_area` $[\mathrm{N/(m/s)^2}]$、 $\mu_{rr}$ が `rolling_resistance` です。 $\tanh$ は停止時の符号反転を滑らかにするためのものです。

---

## 6. キネマティック／ダイナミック ブレンド

$\lambda(v_x)$ を低速側 0、高速側 1 とし、横運動の微分をブレンドします。

$$
\begin{aligned}
\lambda &= \mathrm{clamp}\left(\frac{|v_x| - v_{lo}}{v_{hi} - v_{lo}},\ 0,\ 1\right) \\
\dot v_y &= \lambda\ \dot v_{y,\text{dyn}} + (1 - \lambda)\frac{v_{y,\text{kin}} - v_y}{\tau} \\
\dot r &= \lambda\ \dot r_{\text{dyn}} + (1 - \lambda)\frac{r_{\text{kin}} - r}{\tau}
\end{aligned}
$$

ここで

$$
v_{y,\text{kin}} = v_x \tan\beta_{\text{kin}},
\qquad
r_{\text{kin}} = \frac{v_x \cos\beta_{\text{kin}} \tan\delta}{L}
$$

低速側は「キネマティック解へ一次遅れで引き込む」形で、単輪モデルの $1/v_x$ 特異点を回避しつつ、高速側では純粋な動力学に戻ります。

---

## 7. ダブルトラック（4輪）

輪別スリップ角（左旋回 $\delta_l > \delta_r$）：

$$
\begin{aligned}
\alpha_{fl} &= \delta_l - \arctan\frac{v_y + l_f r}{v_x - T_f r/2},
&\qquad
\alpha_{fr} &= \delta_r - \arctan\frac{v_y + l_f r}{v_x + T_f r/2} \\
\alpha_{rl} &= -\arctan\frac{v_y - l_r r}{v_x - T_r r/2},
&\qquad
\alpha_{rr} &= -\arctan\frac{v_y - l_r r}{v_x + T_r r/2}
\end{aligned}
$$

荷重移動（前後 + 左右、ロール剛性配分 $k_f$）：

$$
\begin{aligned}
F_{zf,\text{axle}} &= \frac{m g l_r}{L} - \frac{m a_x h}{L},
&\qquad
F_{zr,\text{axle}} &= \frac{m g l_f}{L} + \frac{m a_x h}{L} \\
\Delta F_{z,\text{front}} &= \frac{m a_y h k_f}{T_f},
&\qquad
\Delta F_{z,\text{rear}} &= \frac{m a_y h (1 - k_f)}{T_r}
\end{aligned}
$$

1パス予測子で解いています（静荷重で力を計算 → $a_x, a_y$ を推定 → 荷重を再計算 → 力を再計算）。反復しないので WCET が見積もれます。

ヨーモーメントには縦力によるモーメントも含みます：

$$
M_z = l_f\left(F_{y,fl} + F_{y,fr}\right) - l_r\left(F_{y,rl} + F_{y,rr}\right) +
\frac{T_f}{2}\left(F_{x,fr} - F_{x,fl}\right) +
\frac{T_r}{2}\left(F_{x,rr} - F_{x,rl}\right)
$$

（ $F_x, F_y$ は車体座標へ回転済み）。トルクベクタリングや左右制動力差によるヨー制御を扱えるのはこのモデルからです。

---

## 8. タイヤモデル

**線形**：

$$
F_y = C_\alpha \alpha,
\qquad |F_y| \le \mu F_z \text{ で飽和}
$$

**Fiala（ブラシ）**：すべり角

$$
\alpha_{sl} = \arctan\frac{3\mu F_z}{C_\alpha}
$$

まで3次で立ち上がり、以降は $\mu F_z$ 一定。

$$
F_y = C_\alpha \tan\alpha -
\frac{C_\alpha^2}{3\mu F_z}\left|\tan\alpha\right|\tan\alpha +
\frac{C_\alpha^3}{27\mu^2 F_z^2}\tan^3\alpha
$$

**Pacejka Magic Formula（純横力）**：

$$
F_y = D \sin\Big(C \arctan\big(B\alpha - E(B\alpha - \arctan B\alpha)\big)\Big),
\qquad D = \mu F_z
$$

$\alpha = 0$ での傾き（コーナリングスティフネス）は

$$
\left.\frac{\partial F_y}{\partial \alpha}\right|_{\alpha=0} = BCD
$$

`PacejkaTire::fromCorneringStiffness()` は指定した $C_\alpha$ と公称荷重に合う $B$ を逆算するので、線形モデルとの差し替え比較ができます。

**複合スリップ**：摩擦楕円によるスケーリング

$$
F_y \leftarrow F_y \sqrt{1 - \left(\frac{F_x}{\mu F_z}\right)^2}
$$

---

## 9. 積分器

`stepEuler`（1次）、`stepHeun`（2次）、`stepRK4`（4次）。局所打ち切り誤差は

$$
e_{\text{Euler}} = \mathcal{O}(h^2),
\qquad
e_{\text{Heun}} = \mathcal{O}(h^3),
\qquad
e_{\text{RK4}} = \mathcal{O}(h^5)
$$

で、大域誤差は各々 $\mathcal{O}(h)$、 $\mathcal{O}(h^2)$、 $\mathcal{O}(h^4)$。`test_integrator.cpp` では四分円軌道で刻み幅を半分にしたときの誤差比

$$
\frac{e(h)}{e(h/2)} \approx 2,\ 4,\ 16
$$

を確認しています。制御周期 10 ms 程度なら RK4 で十分、実機の固定小数点実装を模す場合は Euler を選んでください。
