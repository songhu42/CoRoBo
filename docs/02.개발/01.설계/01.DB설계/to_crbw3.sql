-- --------------------------------------------------------
-- 호스트:                          songhu.co.kr
-- 서버 버전:                        5.6.41 - MySQL Community Server (GPL)
-- 서버 OS:                        Linux
-- HeidiSQL 버전:                  12.8.0.6908
-- --------------------------------------------------------

/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET NAMES utf8 */;
/*!50503 SET NAMES utf8mb4 */;
/*!40103 SET @OLD_TIME_ZONE=@@TIME_ZONE */;
/*!40103 SET TIME_ZONE='+00:00' */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;
/*!40111 SET @OLD_SQL_NOTES=@@SQL_NOTES, SQL_NOTES=0 */;

-- 테이블 siptdb.ADMIN_COOKIE 구조 내보내기
CREATE TABLE IF NOT EXISTS `ADMIN_COOKIE` (
  `ADMIN_ID` varchar(50) NOT NULL DEFAULT '',
  `AUTO_LOGIN_KEY` varchar(50) NOT NULL DEFAULT '',
  `REMOTE_ADDR` varchar(50) DEFAULT NULL,
  `REG_DT` datetime DEFAULT NULL,
  PRIMARY KEY (`ADMIN_ID`,`AUTO_LOGIN_KEY`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;

-- 테이블 데이터 siptdb.ADMIN_COOKIE:~3 rows (대략적) 내보내기
INSERT INTO `ADMIN_COOKIE` (`ADMIN_ID`, `AUTO_LOGIN_KEY`, `REMOTE_ADDR`, `REG_DT`) VALUES
	('songhu', '0cvkFl51SkhGCG8w6vP0yA==', '0:0:0:0:0:0:0:1', '2024-10-02 14:18:41'),
	('songhu', 'DkZJaY8gLq14J8qs0Y8J3Q==', '0:0:0:0:0:0:0:1', '2024-10-10 16:49:35'),
	('songhu', 'QtLtlB7/csJgauEEnkX2Ow==', '0:0:0:0:0:0:0:1', '2024-10-15 15:49:03');

-- 테이블 siptdb.ADMIN_MENU 구조 내보내기
CREATE TABLE IF NOT EXISTS `ADMIN_MENU` (
  `MENU_ID` varchar(50) NOT NULL DEFAULT '',
  `MENU_NM` varchar(50) DEFAULT NULL,
  `GRP_NM` varchar(50) DEFAULT NULL,
  `GRP_SORTS` int(11) DEFAULT NULL,
  `SORTS` int(11) DEFAULT NULL,
  `CONT_TYPE` varchar(10) DEFAULT 'F',
  `CONTENT` varchar(2000) DEFAULT NULL,
  `AUTH_LEVEL` varchar(10) DEFAULT 'U',
  `IS_USED` int(11) DEFAULT '1',
  `OPEN_AREA` varchar(10) DEFAULT 'C',
  `POPUP_OPT` varchar(100) DEFAULT NULL,
  `REMRKS` varchar(255) DEFAULT NULL,
  `REG_DT` datetime DEFAULT NULL,
  PRIMARY KEY (`MENU_ID`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;

-- 테이블 데이터 siptdb.ADMIN_MENU:~12 rows (대략적) 내보내기
INSERT INTO `ADMIN_MENU` (`MENU_ID`, `MENU_NM`, `GRP_NM`, `GRP_SORTS`, `SORTS`, `CONT_TYPE`, `CONTENT`, `AUTH_LEVEL`, `IS_USED`, `OPEN_AREA`, `POPUP_OPT`, `REMRKS`, `REG_DT`) VALUES
	('AM_101', '어드민 메뉴관리', '마스터관리', 1, 1, 'F', 'contAdminMenu.jsp', 'M', 1, 'C', '', '최초 로딩 메뉴', '2024-10-16 16:46:36'),
	('AM_104', '패스워드변경', '마스터관리', 1, 4, 'F', 'pwchange.jsp', 'O', 1, 'C', '', '', '2024-10-16 16:46:36'),
	('AM_201', '수업일정관리', '수업관리', 2, 1, 'F', 'contCalendarView.jsp', 'O', 1, 'C', '', '', '2024-10-16 16:46:36'),
	('AM_202', '사용자관리', '수업관리', 2, 2, 'F', 'contUserList.jsp', 'O', 1, 'C', '', '', '2024-10-16 16:46:36'),
	('AM_205', '공지사항관리', '수업관리', 2, 5, 'F', 'contBoardAnn.jsp', 'O', 1, 'C', '', '', '2024-10-16 16:46:36'),
	('AM_402', '수업마스터관리', '수업관리', 2, 3, 'F', 'contClassList.jsp', 'U', 1, 'C', '', '', '2024-10-16 16:46:36'),
	('AM_403', '상세수업관리', '수업관리', 2, 4, 'F', 'contClassInfoList.jsp', 'U', 1, 'C', '', '', '2024-10-16 16:46:36'),
	('AM_501', '일자별 매출현황', '수업통계', 5, 1, 'F', 'contGoegDailySales.jsp', 'U', 1, 'C', '', '', '2024-10-16 16:46:36'),
	('AM_502', '일자별 매출 그래프', '수업통계', 5, 2, 'F', 'contGoegDailyGraph.jsp', 'U', 1, 'C', '', '', '2024-10-16 16:46:36'),
	('AM_503', '주간KPI현황', '수업통계', 5, 3, 'F', 'contGoegWeeklyKpi.jsp', 'U', 1, 'C', '', '', '2024-10-16 16:46:36'),
	('AM_901', '컨텐츠 테스트(HTML)', '테스트메뉴', 9, 1, 'C', '<h1><font color=\'red\'>하하하 <br/>\r\n나는 나다!! </font></h1>', 'S', 1, 'P', '', '', '2024-10-16 16:46:36'),
	('AM_902', '컨텐츠 테스트(외부)', '테스트메뉴', 9, 2, 'L', 'http://humanval.cafe24.com/xe/', 'U', 1, 'C', '', '', '2024-10-16 16:46:36');

-- 테이블 siptdb.ADMIN_MST 구조 내보내기
CREATE TABLE IF NOT EXISTS `ADMIN_MST` (
  `ADMIN_ID` varchar(50) NOT NULL DEFAULT '',
  `ADMIN_NM` varchar(50) DEFAULT NULL,
  `TEL_NO` varchar(50) DEFAULT NULL,
  `EMAIL` varchar(255) DEFAULT NULL,
  `AUTO_LOGIN` int(11) DEFAULT NULL,
  `PASSWD` varchar(100) DEFAULT NULL,
  `AUTH_LEVEL` varchar(50) DEFAULT NULL,
  `REG_DT` datetime DEFAULT NULL,
  PRIMARY KEY (`ADMIN_ID`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;

-- 테이블 데이터 siptdb.ADMIN_MST:~1 rows (대략적) 내보내기
INSERT INTO `ADMIN_MST` (`ADMIN_ID`, `ADMIN_NM`, `TEL_NO`, `EMAIL`, `AUTO_LOGIN`, `PASSWD`, `AUTH_LEVEL`, `REG_DT`) VALUES
	('songhu', '송훈구', '010-2272-1796', 'songhu@daum.net', 0, 'mjDCICZngXuFggDSy9Gmhg==', 'S', '2024-09-24 12:31:23');

-- 테이블 siptdb.COM_CODE 구조 내보내기
CREATE TABLE IF NOT EXISTS `COM_CODE` (
  `PCODE` varchar(10) NOT NULL DEFAULT '',
  `CODE` varchar(10) NOT NULL DEFAULT '',
  `PTYPE` varchar(10) DEFAULT NULL,
  `NAME` varchar(50) DEFAULT NULL,
  `ENAME` varchar(50) DEFAULT NULL,
  `SORTS` int(11) DEFAULT '0',
  `REMRK` varchar(2000) DEFAULT NULL,
  `REG_DT` datetime DEFAULT NULL,
  PRIMARY KEY (`PCODE`,`CODE`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;

-- 테이블 데이터 siptdb.COM_CODE:~102 rows (대략적) 내보내기
INSERT INTO `COM_CODE` (`PCODE`, `CODE`, `PTYPE`, `NAME`, `ENAME`, `SORTS`, `REMRK`, `REG_DT`) VALUES
	('A', 'A09', 'COM', '관리자 권한', 'Admin Auth Level', 9, '사용자, 운영자, 관리자, 시스템', '2018-10-23 11:32:39'),
	('A', 'A10', 'COM', '메뉴 컨텐츠 구분', 'Menu Content Type', 10, 'Inner File, HTML Content, Outer Link', '2018-10-23 11:32:40'),
	('A', 'A11', 'COM', '메뉴 열기 구분', 'Menu Open Area', 11, 'Content Area, Popup', '2018-10-23 11:32:40'),
	('A', 'A12', 'COM', '공지종류', 'Announce Type', 10, '전체, 피드백, 개인공지', '2018-10-24 10:17:53'),

	('A09', 'M', 'COM', '관리자', 'Manager', 3, '', '2018-10-23 11:32:39'),
	('A09', 'O', 'COM', '운영자', 'Operator', 2, '', '2018-10-23 11:32:39'),
	('A09', 'S', 'COM', '시스템', 'System', 4, '', '2018-10-23 11:32:39'),
	('A09', 'U', 'COM', '사용자', 'User', 1, '', '2018-10-23 11:32:39'),
	('A10', 'C', 'COM', 'HTML 컨텐츠', 'HTML Content', 2, '', '2018-10-23 11:32:40'),
	('A10', 'F', 'COM', '내부 파일', 'Inner File', 1, '', '2018-10-23 11:32:40'),
	('A10', 'L', 'COM', '외부 링크', 'Outer Link', 3, '', '2018-10-23 11:32:40'),
	('A11', 'C', 'COM', '컨텐츠 영역', 'Content Area', 1, '', '2018-10-23 11:33:03'),
	('A11', 'P', 'COM', '팝업', 'Popup', 2, '', '2018-10-23 11:33:03'),
	('A12', 'A', 'COM', '전체공지', 'Announce All', 1, '', '2018-10-24 10:17:54'),
	('A12', 'F', 'COM', '피드백응답', 'Response Feedback', 2, '', '2018-10-24 10:17:54'),
	('A12', 'P', 'COM', '개인공지', 'Personal Ann', 3, '', '2018-10-24 10:17:54') 
;

/*!40103 SET TIME_ZONE=IFNULL(@OLD_TIME_ZONE, 'system') */;
/*!40101 SET SQL_MODE=IFNULL(@OLD_SQL_MODE, '') */;
/*!40014 SET FOREIGN_KEY_CHECKS=IFNULL(@OLD_FOREIGN_KEY_CHECKS, 1) */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40111 SET SQL_NOTES=IFNULL(@OLD_SQL_NOTES, 1) */;
