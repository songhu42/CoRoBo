import sys
import time
import mysql.connector
import rclpy

logger = None

# MySQL 서버에 연결
def connect_db(node_logger=None):
    global logger 
    if node_logger != None :
        logger = node_logger

    conn = mysql.connector.connect(
        host='192.168.0.17',     
        user='corobo',    
        passwd='corobo12',   
        database='corobodb'    
    )
    return conn


def select_where(conn, tb_name, whereOption):
    global logger 
    # 커서 생성
    cursor = conn.cursor()

    # 쿼리 실행 예시
    sql_query = f"SELECT * FROM {tb_name} {whereOption}" 
    cursor.execute(sql_query)

    return cursor 


def test_db(node_logger=None):
    global logger 

    if node_logger != None :
        logger = node_logger

    log(f"testDB() start ... ")
    try:
        conn = connect_db()
        log(f"connectDB {conn}")
        # 쿼리 결과 가져오기
        cursor = select_where(conn, "ADMIN_MENU", "")
        log(f"select_where {cursor}")

        result = cursor.fetchall()
        for row in result:
            log(f"menu_id : {row[0]} menu_nm : {row[1]} ")

    except Exception as e:
        log(f"DB Exception !! => {e.__doc__}")
        log(e.__doc__)
        log(e.__traceback__)

    finally:
        # 연결과 커서 닫기
        cursor.close()
        conn.close()

def log(msg):
    global logger 

    if logger == None:
        print(msg)
    else :
        logger.info(msg)

def main(args=None):
    log("main start... ")
    
    # db test 
    test_db()

if __name__ == "__main__":
    main()
