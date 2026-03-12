import sys
import logging
from logging.handlers import RotatingFileHandler
import subprocess
import threading
from pathlib import Path
from typing import List


def init_logger(name, log_dir=Path.home()/"logs"):
    log_dir = Path(log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    # 1. 创建logger对象
    logger = logging.getLogger(name)

    # 2. 设置日志等级
    logger.setLevel(logging.DEBUG)

    # 3. 创建日志处理器
    log_file = Path(log_dir) / f'{name}.log'
    
    # 禁止日志冒泡
    logger.propagate = False
        
    # 使用'w'模式创建新文件而不是追加到现有文件
    fh = RotatingFileHandler(log_file, mode='w', maxBytes=5*1024*1024, backupCount=5)
    # ch = logging.StreamHandler(sys.stdout)

    # 4. 创建日志格式
    formatter = logging.Formatter(
        '[%(levelname)s] [%(asctime)s] [%(name)s]: %(message)s',
        datefmt='%m-%d %H:%M:%S'
    )
    fh.setFormatter(formatter)
    # ch.setFormatter(formatter)

    # 5. 添加日志处理器
    logger.addHandler(fh)
    # logger.addHandler(ch)


def read_stream(stream, logger=None, level=logging.INFO, prefix=""):
    """
    subprocess时读取stdout流并写入日志
    :param stream: 输出流
    :param logger: 日志记录器，如果为None则直接打印
    :param level: 日志级别
    :param prefix: 日志前缀
    :return:
    """
    try:
        for line in iter(stream.readline, ''):
            line = line.strip()
            if line:  # 只记录非空行
                if logger:
                    # 线程安全地写入日志
                    logger.log(level, f"{prefix}{line}")
                else:
                    print(f"{prefix}{line}")
    except Exception as e:
        if logger:
            logger.error(f"读取subprocess输出流时出错: {e}")
        else:
            print(f"读取subprocess输出流时出错: {e}")
    finally:
        try:
            stream.close()
        except:
            pass


def launch_python_subprocess(cmd: List[str], read_stdout=True, logger=None, log_prefix="[SUBPROC] "):
    """
    启动Python子进程并可选择将输出导入日志
    
    :param cmd: 要执行的Python命令列表
    :param read_stdout: 是否读取stdout输出
    :param logger: 日志记录器，如果提供则将subprocess输出写入日志
    :param log_prefix: 日志前缀，用于区分subprocess输出
    :return: subprocess.Popen对象
    """
    full_cmd = [sys.executable, "-u"] + cmd  # 使用当前Python解释器执行命令
    process = subprocess.Popen(
        full_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,  # 行缓冲，确保及时输出
        universal_newlines=True
    )
    if read_stdout:
        # 只需要一个线程读取合并后的输出
        output_thread = threading.Thread(
            target=read_stream, 
            args=(process.stdout, logger, logging.INFO, log_prefix), 
            daemon=True,
            name=f"subprocess-output-{process.pid}"
        )
        output_thread.start()

    return process    



def launch_subprocess(cmd: List[str], read_stdout=True, logger=None, log_prefix="[SUBPROC] "):
    """
    启动子进程
    
    :param cmd: 要执行的命令列表
    :param read_stdout: 是否读取输出
    :param logger: 日志记录器
    :param log_prefix: 日志前缀
    :return: subprocess.Popen对象
    """
    process = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,  # 将stderr重定向到stdout，合并输出
        text=True,
        bufsize=1,
        universal_newlines=True
    )

    if read_stdout:
        # 只需要一个线程读取合并后的输出
        output_thread = threading.Thread(
            target=read_stream, 
            args=(process.stdout, logger, logging.INFO, log_prefix), 
            daemon=True,
            name=f"subprocess-output-{process.pid}"
        )
        output_thread.start()

    return process    
    