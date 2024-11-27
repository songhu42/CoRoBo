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

-- 내보낼 데이터가 선택되어 있지 않습니다.

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

-- 내보낼 데이터가 선택되어 있지 않습니다.

/*!40103 SET TIME_ZONE=IFNULL(@OLD_TIME_ZONE, 'system') */;
/*!40101 SET SQL_MODE=IFNULL(@OLD_SQL_MODE, '') */;
/*!40014 SET FOREIGN_KEY_CHECKS=IFNULL(@OLD_FOREIGN_KEY_CHECKS, 1) */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40111 SET SQL_NOTES=IFNULL(@OLD_SQL_NOTES, 1) */;
