# Báo cáo LaTeX

File chính: `reports/uav_oda_report.tex`

Biên dịch khuyến nghị:

```bash
xelatex reports/uav_oda_report.tex
xelatex reports/uav_oda_report.tex
```

Nếu chạy từ bên trong thư mục `reports/`, dùng:

```bash
cd reports
xelatex uav_oda_report.tex
xelatex uav_oda_report.tex
```

Nếu dùng Overleaf, upload cả thư mục `reports/` và `outputs/figures/`, đặt compiler là **XeLaTeX**.

Cần sửa nhanh các macro ở đầu file:

```tex
\newcommand{\SinhVien}{Nguyễn Thanh Lâm}
\newcommand{\Mentor}{Hoàng Công Phát}
\newcommand{\DonViMentor}{Viettel High Tech}
\newcommand{\EmailMentor}{}
```
