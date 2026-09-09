#!/usr/bin/env python3
from pathlib import Path
from tempfile import NamedTemporaryFile
from zipfile import ZIP_DEFLATED, ZipFile

from docx import Document
from docx.enum.section import WD_SECTION
from docx.enum.style import WD_STYLE_TYPE
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Inches, Pt, RGBColor


OUTPUT = Path("/Users/lichengyan/Documents/ChatGPT/碩士論文/output/SP專案任務實作總結_草稿.docx")
FONT = "Arial Unicode MS"
BLACK = RGBColor(0x00, 0x00, 0x00)
BLUE = RGBColor(0x18, 0x4E, 0x77)
GRAY = RGBColor(0x55, 0x5F, 0x68)


def set_run_font(run, size=None, bold=None, color=None):
    run.font.name = FONT
    r_pr = run._element.get_or_add_rPr()
    r_pr.rFonts.set(qn("w:ascii"), FONT)
    r_pr.rFonts.set(qn("w:hAnsi"), FONT)
    r_pr.rFonts.set(qn("w:eastAsia"), FONT)
    r_pr.rFonts.set(qn("w:hint"), "eastAsia")
    lang = r_pr.find(qn("w:lang"))
    if lang is None:
        lang = OxmlElement("w:lang")
        r_pr.append(lang)
    lang.set(qn("w:val"), "zh-TW")
    lang.set(qn("w:eastAsia"), "zh-TW")
    if size is not None:
        run.font.size = Pt(size)
    if bold is not None:
        run.bold = bold
    if color is not None:
        run.font.color.rgb = color


def set_repeat_font(style, size, bold=False, color=BLACK):
    style.font.name = FONT
    r_pr = style._element.get_or_add_rPr()
    r_pr.rFonts.set(qn("w:ascii"), FONT)
    r_pr.rFonts.set(qn("w:hAnsi"), FONT)
    r_pr.rFonts.set(qn("w:eastAsia"), FONT)
    r_pr.rFonts.set(qn("w:hint"), "eastAsia")
    lang = r_pr.find(qn("w:lang"))
    if lang is None:
        lang = OxmlElement("w:lang")
        r_pr.append(lang)
    lang.set(qn("w:val"), "zh-TW")
    lang.set(qn("w:eastAsia"), "zh-TW")
    style.font.size = Pt(size)
    style.font.bold = bold
    style.font.color.rgb = color


def add_page_number(paragraph):
    paragraph.alignment = WD_ALIGN_PARAGRAPH.RIGHT
    run = paragraph.add_run("第 ")
    set_run_font(run, 8, color=GRAY)
    fld_char_1 = OxmlElement("w:fldChar")
    fld_char_1.set(qn("w:fldCharType"), "begin")
    instr_text = OxmlElement("w:instrText")
    instr_text.set(qn("xml:space"), "preserve")
    instr_text.text = " PAGE "
    fld_char_2 = OxmlElement("w:fldChar")
    fld_char_2.set(qn("w:fldCharType"), "end")
    run._r.append(fld_char_1)
    run._r.append(instr_text)
    run._r.append(fld_char_2)
    run2 = paragraph.add_run(" 頁")
    set_run_font(run2, 8, color=GRAY)


def add_section_heading(doc, text):
    p = doc.add_paragraph(style="Section Label")
    p.add_run(text)
    return p


def add_body(doc, text, bold_lead=None):
    p = doc.add_paragraph(style="Body Compact")
    if bold_lead and text.startswith(bold_lead):
        lead = p.add_run(bold_lead)
        set_run_font(lead, 10.4, bold=True, color=BLACK)
        rest = p.add_run(text[len(bold_lead):])
        set_run_font(rest, 10.4, color=BLACK)
    else:
        r = p.add_run(text)
        set_run_font(r, 10.4, color=BLACK)
    return p


def add_bullet(doc, text, bold_lead=None):
    p = doc.add_paragraph(style="Bullet Compact")
    if bold_lead and text.startswith(bold_lead):
        lead = p.add_run(bold_lead)
        set_run_font(lead, 10.1, bold=True, color=BLACK)
        rest = p.add_run(text[len(bold_lead):])
        set_run_font(rest, 10.1, color=BLACK)
    else:
        r = p.add_run(text)
        set_run_font(r, 10.1, color=BLACK)
    return p


def add_task_header(doc, code, title, status, summary, page_break=False):
    p_code = doc.add_paragraph(style="Task Code")
    if page_break:
        p_code.paragraph_format.page_break_before = True
    p_code.add_run(code)
    p_title = doc.add_paragraph(style="Heading 1")
    p_title.add_run(title)
    p_status = doc.add_paragraph(style="Status Line")
    p_status.add_run(status)
    add_body(doc, summary)


def configure_document(doc):
    section = doc.sections[0]
    section.page_width = Inches(8.5)
    section.page_height = Inches(11)
    section.top_margin = Inches(0.58)
    section.bottom_margin = Inches(0.56)
    section.left_margin = Inches(0.74)
    section.right_margin = Inches(0.74)
    section.header_distance = Inches(0.25)
    section.footer_distance = Inches(0.25)

    normal = doc.styles["Normal"]
    set_repeat_font(normal, 10.4)
    normal.paragraph_format.space_after = Pt(4)
    normal.paragraph_format.line_spacing = 1.18

    title = doc.styles["Title"]
    set_repeat_font(title, 24, bold=True)
    title.paragraph_format.space_after = Pt(3)
    title.paragraph_format.line_spacing = 1.0

    heading1 = doc.styles["Heading 1"]
    set_repeat_font(heading1, 19, bold=True)
    heading1.paragraph_format.space_before = Pt(0)
    heading1.paragraph_format.space_after = Pt(4)
    heading1.paragraph_format.keep_with_next = True

    styles = doc.styles

    subtitle = styles.add_style("Document Subtitle", WD_STYLE_TYPE.PARAGRAPH)
    set_repeat_font(subtitle, 10.5, color=GRAY)
    subtitle.paragraph_format.space_after = Pt(8)

    intro = styles.add_style("Document Intro", WD_STYLE_TYPE.PARAGRAPH)
    set_repeat_font(intro, 9.5, color=GRAY)
    intro.paragraph_format.space_after = Pt(10)
    intro.paragraph_format.line_spacing = 1.15

    code = styles.add_style("Task Code", WD_STYLE_TYPE.PARAGRAPH)
    set_repeat_font(code, 9, bold=True, color=BLUE)
    code.paragraph_format.space_after = Pt(2)
    code.paragraph_format.keep_with_next = True

    status = styles.add_style("Status Line", WD_STYLE_TYPE.PARAGRAPH)
    set_repeat_font(status, 10.2, bold=True, color=BLUE)
    status.paragraph_format.space_after = Pt(5)
    status.paragraph_format.keep_with_next = True

    section_label = styles.add_style("Section Label", WD_STYLE_TYPE.PARAGRAPH)
    set_repeat_font(section_label, 11.1, bold=True, color=BLACK)
    section_label.paragraph_format.space_before = Pt(6)
    section_label.paragraph_format.space_after = Pt(2)
    section_label.paragraph_format.keep_with_next = True

    body = styles.add_style("Body Compact", WD_STYLE_TYPE.PARAGRAPH)
    set_repeat_font(body, 10.4)
    body.paragraph_format.space_after = Pt(4)
    body.paragraph_format.line_spacing = 1.18

    bullet = styles.add_style("Bullet Compact", WD_STYLE_TYPE.PARAGRAPH)
    set_repeat_font(bullet, 10.1)
    bullet.paragraph_format.left_indent = Inches(0.22)
    bullet.paragraph_format.first_line_indent = Inches(-0.16)
    bullet.paragraph_format.space_after = Pt(2.5)
    bullet.paragraph_format.line_spacing = 1.14

    footer = section.footer
    fp = footer.paragraphs[0]
    add_page_number(fp)


def ensure_font_table_entry(path):
    """Declare the selected font so LibreOffice honors East Asian run fonts."""
    with ZipFile(path, "r") as zin:
        font_xml = zin.read("word/fontTable.xml")
        root = OxmlElement("w:fonts")
        from lxml import etree

        root = etree.fromstring(font_xml)
        font_names = {
            node.get(qn("w:name")) for node in root.findall(qn("w:font"))
        }
        if FONT not in font_names:
            font = OxmlElement("w:font")
            font.set(qn("w:name"), FONT)
            alt = OxmlElement("w:altName")
            alt.set(qn("w:val"), FONT)
            charset = OxmlElement("w:charset")
            charset.set(qn("w:val"), "01")
            family = OxmlElement("w:family")
            family.set(qn("w:val"), "swiss")
            pitch = OxmlElement("w:pitch")
            pitch.set(qn("w:val"), "variable")
            font.extend([alt, charset, family, pitch])
            root.append(font)
        updated = etree.tostring(
            root, xml_declaration=True, encoding="UTF-8", standalone=True
        )

        with NamedTemporaryFile(delete=False, suffix=".docx") as temp_file:
            temp_path = Path(temp_file.name)
        with ZipFile(temp_path, "w", ZIP_DEFLATED) as zout:
            for item in zin.infolist():
                data = updated if item.filename == "word/fontTable.xml" else zin.read(item.filename)
                zout.writestr(item, data)
    temp_path.replace(path)


def build_document():
    doc = Document()
    configure_document(doc)
    doc.core_properties.title = "SP 專案任務實作總結"
    doc.core_properties.subject = "Isaac Sim SO 101 Shared Autonomy"
    doc.core_properties.author = "李承諺"
    doc.core_properties.keywords = "Isaac Sim, Isaac Lab, LeIsaac, SO-101, WebSocket, Differential IK"

    p = doc.add_paragraph(style="Title")
    p.add_run("SP 專案任務實作總結")
    p = doc.add_paragraph(style="Document Subtitle")
    p.add_run("李承諺  2026 年 9 月  Isaac Sim SO 101 Shared Autonomy")
    p = doc.add_paragraph(style="Document Intro")
    p.add_run(
        "本文件整理我目前在 SP 中實際完成或參與的四項任務。S 1 建立模擬基礎，"
        "L 1 定義通訊封包，N 1 驗證遠端訊息傳輸，S 2 完成末端位置控制。"
        "各頁只記錄已有操作紀錄或測試輸出支持的結果，並標明尚未完成的整合範圍。"
    )

    add_task_header(
        doc,
        "S 1",
        "SO 101 模擬環境建置與關節驗證",
        "狀態  已完成",
        "我在 RTXWS 的 Isaac Sim 容器內完成 SO 101 資產匯入、Isaac Lab 與 LeIsaac 安裝，並確認六個關節可被程式辨識與控制。這項工作建立了後續逆向運動學與遠端操控所需的模擬介面。",
    )
    add_section_heading(doc, "任務目標")
    add_body(doc, "建立可重現的 SO 101 模擬環境，確認機器人模型能載入、關節名稱與索引正確，且至少一個控制範例可穩定執行並留下畫面與紀錄。")
    add_section_heading(doc, "完成內容")
    add_bullet(doc, "建立持久化工作目錄於 /root/Documents/robotics，區分 upstream、work 與 data，避免程式與紀錄只存在容器暫存層。")
    add_bullet(doc, "確認 Isaac Sim 5.0.0、內建 Python 3.11.13、PyTorch 2.7.0 與 CUDA 12.8 可使用，並安裝 LeIsaac 0.4.0 及其 Isaac Lab 2.3.0 相依套件。")
    add_bullet(doc, "下載並載入 so101_follower.usd，辨識 shoulder pan、shoulder lift、elbow flex、wrist flex、wrist roll 與 gripper 共六個關節。")
    add_bullet(doc, "撰寫肩部旋轉測試，使 shoulder pan 在正負 10 度之間往返；模擬畫面與輸出顯示實際角度會跟隨目標角度。")
    add_section_heading(doc, "問題與處理")
    add_body(doc, "原始容器沒有全域 python3、git、cmake 與 nano，Isaac Lab 也尚未加入 Isaac Sim 的 Python 環境。我改用 /isaac-sim/python.sh，建立 _isaac_sim 連結並完成 editable install，之後以 Isaac Lab 啟動腳本執行測試。")
    add_section_heading(doc, "驗證結果")
    add_bullet(doc, "SO 101 已在 Isaac Sim 畫面中正常顯示，WebRTC 遠端畫面可觀察關節運動。", "SO 101")
    add_bullet(doc, "shoulder pan 的正負 10 度往返測試可連續執行；其餘關節也能接受目標值。肩部抬升與手肘的追蹤誤差較大，主要與重力及致動器參數有關，因此本任務只確認控制介面與基本運動，不宣稱已完成動力學調校。", "shoulder pan")
    add_section_heading(doc, "任務產出")
    add_body(doc, "完成啟動步驟、測試程式、終端輸出與模擬截圖。S 1 的環境與關節索引直接作為 S 2 末端位置控制的基礎。")

    add_task_header(
        doc,
        "L 1",
        "WebSocket 訊息封包契約",
        "狀態  與 Thomas 共同完成初版並進入審核",
        "我與 Thomas 先固定手機端與伺服器端共用的訊息外包裝，使雙方能分工開發而不必等待另一端完成。這項任務定義的是資料交換契約，不包含手機 App 實作或機器手臂控制。",
        page_break=True,
    )
    add_section_heading(doc, "任務目標")
    add_body(doc, "建立最小且可擴充的訊息格式，讓手機 App、Mac 中繼與 RTXWS 伺服器對版本、時間、順序與資料內容有一致理解，並能對無效訊息給出明確錯誤。")
    add_section_heading(doc, "封包結構")
    add_bullet(doc, "v 代表合約版本號。格式或欄位規則改變時更新版本，接收端可依版本判斷是否相容。", "v")
    add_bullet(doc, "type 表示訊息用途，例如 hello、welcome、test、error，後續可加入 move delta 等控制指令。", "type")
    add_bullet(doc, "ts 是寄出端以毫秒表示的時間戳，可用於延遲分析與事件排序；seq 是流水號，可協助辨識重送或過期指令。", "ts")
    add_bullet(doc, "payload 放置各 type 專屬資料，使共同欄位固定，內容則依訊息用途擴充。", "payload")
    add_section_heading(doc, "共同規則")
    add_bullet(doc, "MVP 採用同一個 Wi Fi、WebSocket 與 JSON 純文字。未來若改用 MQTT，只替換傳輸管道，訊息內容可沿用。")
    add_bullet(doc, "接收端遇到未知 type、版本不支援、必要欄位缺漏或 JSON 無法解析時，應拒絕訊息並回傳 error，不讓錯誤控制命令進入機器人端。")
    add_bullet(doc, "seq 需由送出端單調增加；伺服器可丟棄已處理或過期的序號，降低重複命令造成手臂抖動的風險。")
    add_section_heading(doc, "分工與驗證方式")
    add_body(doc, "Thomas 依契約製作手機端訊息，我負責伺服器接收、驗證、紀錄與後續模擬控制。N 1 已用 hello、welcome、test 與 error 驗證初版規則，證明契約可實際落地。")
    add_section_heading(doc, "任務邊界")
    add_body(doc, "目前完成的是 v0 封包與傳輸選擇。move delta 的欄位、單位、速率限制與安全規則仍需在手機端與 S 2 整合時共同確認，審核後再凍結為下一版。")

    add_task_header(
        doc,
        "N 1",
        "WebSocket 伺服器與遠端連線驗證",
        "狀態  伺服器與 Mac 通訊已完成  手機端尚未驗證",
        "我建立 WebSocket 伺服器 v0，完成正常訊息、錯誤訊息與 JSONL 紀錄測試，並以 SSH tunnel 驗證 Mac 可把訊息送到 RTXWS 容器。這證明通訊路徑可用，但尚未包含 Thomas 的手機 App。",
        page_break=True,
    )
    add_section_heading(doc, "任務目標")
    add_body(doc, "讓外部裝置能以 L 1 封包連到 RTXWS 的控制伺服器，並留下可追查的收發紀錄。由於 8765 連接埠無法直接從 Mac 存取，本任務也要建立安全且可重現的轉送方式。")
    add_section_heading(doc, "完成內容")
    add_bullet(doc, "建立 server_v0.py，使用 websockets 12.0 監聽 0.0.0.0 的 TCP 8765 連接埠，並以 nohup 在容器背景持續執行。")
    add_bullet(doc, "hello 訊息會取得 welcome、session UUID，以及 coarse 2.0 cm 與 fine 0.5 cm 設定；test 訊息會原樣 echo。")
    add_bullet(doc, "unknown type 會回傳 unknown_type，無效 JSON 會回傳 invalid_json；所有收發方向、時間、來源與內容寫入 server_v0.jsonl。")
    add_bullet(doc, "在 Mac 建立 SSH local forwarding，使 ws://192.168.0.103:8765 經由 Mac 轉送到 leo@140.115.54.63，再進入容器內的 127.0.0.1:8765。")
    add_section_heading(doc, "問題與處理")
    add_body(doc, "Mac 直接連線 140.115.54.63:8765 會逾時，原因是服務位於遠端主機與容器環境，且外部防火牆或網路路徑沒有開放該連接埠。我改用既有 SSH 連線建立 tunnel，不需要把 WebSocket 服務直接暴露到公開網路。")
    add_body(doc, "192.168.0.103 是 Mac 當時在 Wi Fi 中的區域網路位址，換網路後可能改變；140.115.54.63 是 RTXWS 的連線位址；118.163.137.22 則是曾觀察到的對外 NAT 位址，三者用途不同。8765 是本專案選用的 WebSocket TCP 連接埠。")
    add_section_heading(doc, "驗證結果")
    add_bullet(doc, "容器本機測試通過，輸出 N 1 LOCAL TEST PASS 與 N 1 ERROR TEST PASS。")
    add_bullet(doc, "Mac 端收到 welcome 與 test echo，輸出 MAC TO SSH TUNNEL TO RTXWS SERVER TEST PASS；RTXWS 的 JSONL 同時留下對應收發紀錄。")
    add_section_heading(doc, "任務產出與下一步")
    add_body(doc, "完成伺服器、兩支測試客戶端、背景啟動方式、連線檢查與日誌。下一步是將 move delta 訊息接到 S 2，之後再由 Thomas 的手機 App 進行同一 Wi Fi 下的端到端測試。")

    add_task_header(
        doc,
        "S 2",
        "Differential IK 末端位置控制",
        "狀態  已完成模擬控制與安全測試",
        "我使用 Isaac Lab DifferentialIKController 實作 SO 101 的位置型逆向運動學，讓程式以末端位移指定動作，再轉換成五個手臂關節的目標角度。向上與向前 5 cm 均通過誤差門檻，超出工作空間時也能安全停止。",
        page_break=True,
    )
    add_section_heading(doc, "任務目標")
    add_body(doc, "建立可供遠端 move delta 使用的末端位置控制器。輸入以機器人底座座標表示，控制器只調整位置，不要求固定夾爪姿態，並以關節限制、單步角度上限與誤差門檻避免發散。")
    add_section_heading(doc, "控制方法")
    add_bullet(doc, "採用 position 模式與阻尼最小平方法，阻尼係數為 0.05；每一步關節修正限制在 0.01 rad，並將結果限制在 USD 定義的關節範圍內。")
    add_bullet(doc, "PhysX 提供的 Jacobian 原本在世界座標，我先依 root quaternion 旋轉到底座座標，再與同一座標系的末端誤差運算。")
    add_bullet(doc, "加入 gravity hold offset，保留抵抗重力所需的目標角度偏移；若每一幀直接以量測關節角當作新目標，手臂會逐漸下垂並干擾 IK。")
    add_bullet(doc, "命令誤差超過 0.10 m 時停止，成功條件為末端誤差小於 5 mm 且連續維持 60 幀。")
    add_section_heading(doc, "問題與修正")
    add_body(doc, "最初向前測試會向下掉落並在約 0.10 m 誤差時停止。分析後確認主要問題是 Jacobian 與位置誤差使用不同座標系，再加上控制目標缺乏重力補償。完成座標旋轉、重力保持與彎曲初始姿態後，控制結果收斂。WebRTC 啟動時另補裝 h5py，並限制 CUDA_VISIBLE_DEVICES=0，減少多 GPU P2P 初始化問題。")
    add_section_heading(doc, "驗證結果")
    add_bullet(doc, "向上 5 cm 測試的最終誤差為 1.533 mm，最佳誤差 1.043 mm，狀態保持有限值，輸出 S 2 UP 5 CM PASS。", "向上 5 cm")
    add_bullet(doc, "向前 5 cm 測試實際前進約 48.50 mm，最終誤差 3.422 mm，最佳誤差 3.283 mm，並連續 60 幀低於 5 mm，輸出 S 2 FORWARD 5 CM PASS。", "向前 5 cm")
    add_bullet(doc, "不可達目標測試在誤差超過 0.10 m 時觸發 safety stop，數值保持有限，確認失敗情況不會無限制輸出關節命令。", "不可達目標")
    add_bullet(doc, "WebRTC 畫面可觀察手臂重複向前與返回，終端誤差在毫米等級且未觸發安全停止。", "WebRTC")
    add_section_heading(doc, "任務產出與下一步")
    add_body(doc, "完成向上、向前、不可達目標與 WebRTC 顯示四支測試程式。S 2 證明模擬器端能把低維末端位移轉成安全關節命令；下一步是把 N 1 的 move delta 接入此控制器，再與手機 App 驗證完整控制鏈。")

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    doc.save(OUTPUT)
    ensure_font_table_entry(OUTPUT)
    print(OUTPUT)


if __name__ == "__main__":
    build_document()
